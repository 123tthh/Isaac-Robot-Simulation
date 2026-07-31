"""
search_dataspace_no_pitch.py  v4  (Isaac Sim 5.1 · Script Editor)
─────────────────────────────────────────────────────────────────
v4 核心修正：
  ★ set_pallet_pose_usd() 必须在 reset_async() 之后调用
    reset_async 会把场景恢复到初始 USD 状态，覆盖 reset 前的写入
    正确顺序：reset_async → 写位姿 → 几帧 warmup → 推帧读状态

  另外用 physx_interface.set_rigidbody_kinematic_target 直接电传位置到
  PhysX（如果 reset 后 USD 写入仍不生效的兜底方案）。
"""

import os
import math
import json
import asyncio

import numpy as np
import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, UsdPhysics, Gf

# ═══════════════════════════════════════════════════════════════
# ★ 可调参数
# ═══════════════════════════════════════════════════════════════

MU_ROLLER_S_BASE = 0.060
PALLET_MU_S      = 0.400
MU_ROLLER_BASE   = MU_ROLLER_S_BASE * PALLET_MU_S
MU_RAND_RANGE    = 0.30

Y_SEARCH   = np.linspace(0.000, 0.040, 9)
YAW_SEARCH = np.linspace(0.0,   8.0,   9)
Y_GRID     = np.linspace(0.000, 0.040, 7)
YAW_GRID   = np.linspace(0.0,   8.0,   7)

N_TRIALS     = 5
SIM_DURATION = 5.0
PHYSICS_DT   = 1.0 / 60.0
STUCK_SPEED  = 0.005
POSE_SETTLE_STEPS = 5   # reset后写位姿，再空走几帧让PhysX稳定

PAL_X0_ACTUAL  = 0.045
PAL_Z0_ACTUAL  = 1.26751
PAL_Y0_CENTER  = 0.00253
PITCH_BASE     = 4.0

SAVE_PATH   = "/tmp/safe_pose_set_no_pitch.json"
PALLET_PATH = "/World/Pallet/CollisionProxy"

# ───────────────────────────────────────────────────────────────
# 全局句柄
# ───────────────────────────────────────────────────────────────
stage         = None
proxy         = None
pallet_prim   = None
pallet_rb_api = None
world         = None
timeline      = None
app           = None

ROLLER_TOP_Z0  = None
PALLET_X = PALLET_Y = PALLET_H = None
CONVEYOR_END_X = 0.500
SUCCESS_X      = CONVEYOR_END_X * 0.9
LOG            = None


# ═══════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════

def get_scene_baseline():
    r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
    cg  = stage.GetPrimAtPath("/World/ConveyorGroup")
    if not r00.IsValid():
        raise RuntimeError("Roller_00_L 不存在")
    r_z  = r00.GetAttribute("xformOp:translate").Get()[2]
    cg_z = cg.GetAttribute("xformOp:translate").Get()[2] if cg.IsValid() else 0.0
    return r_z + cg_z + 0.010


def set_pallet_pose_usd(y, yaw_deg, pitch_deg=None):
    """
    ★ 必须在 reset_async() 之后调用。
    写 USD 位姿 + 清零速度。
    PhysX 在下一帧会从 USD 读取并同步。
    """
    pitch_deg = pitch_deg if pitch_deg is not None else PITCH_BASE
    xf = UsdGeom.XformCommonAPI(proxy)
    xf.SetTranslate(Gf.Vec3d(PAL_X0_ACTUAL, PAL_Y0_CENTER + y, PAL_Z0_ACTUAL))
    xf.SetRotate(Gf.Vec3f(0.0, float(pitch_deg), float(yaw_deg)),
                 UsdGeom.XformCommonAPI.RotationOrderXYZ)
    if pallet_rb_api:
        pallet_rb_api.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        pallet_rb_api.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))


def _set_roller_mu(mu_s):
    mat = stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return False
    m = UsdPhysics.MaterialAPI.Get(stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(float(mu_s))
        m.GetDynamicFrictionAttr().Set(float(mu_s) * 0.7)
    return True


def randomize_mu():
    mu_eff = MU_ROLLER_BASE * (1.0 + np.random.uniform(-MU_RAND_RANGE, MU_RAND_RANGE))
    mu_eff = float(np.clip(mu_eff, 0.005, 0.065))
    _set_roller_mu(mu_eff / PALLET_MU_S)


def restore_mu():
    _set_roller_mu(MU_ROLLER_S_BASE)


def classify(pos, vel):
    if pos is None:
        return "error"
    x, z  = float(pos[0]), float(pos[2])
    speed = float((vel[0]**2 + vel[1]**2 + vel[2]**2) ** 0.5)
    if x >= SUCCESS_X:
        return "success"
    elif z < PAL_Z0_ACTUAL - 0.15:
        return "fallen"
    elif speed < STUCK_SPEED:
        return "stuck"
    else:
        return "timeout"


# ═══════════════════════════════════════════════════════════════
# 单次试验  ★ 核心修正版
# ═══════════════════════════════════════════════════════════════

async def run_one_trial(y, yaw, randomize=True, verbose=False):
    global pallet_prim

    # ── 1. 停 timeline ───────────────────────────────────────
    if timeline.is_playing():
        timeline.stop()

    # ── 2. 摩擦随机化（stop 状态下改材质，下一次 reset 后生效）
    if randomize:
        randomize_mu()
    else:
        restore_mu()

    # ── 3. reset_async（恢复场景到 USD 初始状态，自动 play）──
    #       注意：这里故意不在 reset 前写位姿，因为 reset 会覆盖
    await world.reset_async()

    # ── 4. ★ reset 之后立刻写位姿 ───────────────────────────
    #       现在物理已经 play 了，直接写 USD 属性，
    #       PhysX 会在下一帧同步（"kinematic teleport" 语义）
    set_pallet_pose_usd(y, yaw_deg=yaw, pitch_deg=PITCH_BASE)

    # ── 5. SingleRigidPrim 也在 reset 后创建/刷新 ─────────
    try:
        from isaacsim.core.prims import SingleRigidPrim
    except ImportError:
        from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
    pallet_prim = SingleRigidPrim(prim_path=PALLET_PATH, name="pallet_trial")

    # ── 6. Settle 帧：让 PhysX 读取新位姿并稳定接触 ────────
    for _ in range(POSE_SETTLE_STEPS):
        await app.next_update_async()

    # 可选：验证位置是否写入成功
    if verbose:
        pos_check, _ = pallet_prim.get_world_pose()
        print(f"      [settle后] PhysX位置: ({float(pos_check[0]):.4f},"
              f" {float(pos_check[1]):.4f}, {float(pos_check[2]):.4f})")

    # ── 7. 正式推帧 ──────────────────────────────────────────
    n_steps = int(SIM_DURATION / PHYSICS_DT)
    log_interval = 60
    for step in range(n_steps):
        await app.next_update_async()
        if verbose and (step + 1) % log_interval == 0:
            pos, _ = pallet_prim.get_world_pose()
            vel    = pallet_prim.get_linear_velocity()
            t = (step + 1) * PHYSICS_DT
            spd = float((vel[0]**2+vel[1]**2+vel[2]**2)**0.5)
            print(f"      {t:.1f}s | X={float(pos[0]):.4f}m | |V|={spd:.5f}m/s")

    # ── 8. 读最终状态 + 停止 ─────────────────────────────────
    pos, _ = pallet_prim.get_world_pose()
    vel    = pallet_prim.get_linear_velocity()
    timeline.stop()
    return classify(pos, vel)


async def run_trials(y, yaw, n=N_TRIALS, verbose=False):
    results = []
    for t in range(n):
        r = await run_one_trial(y, yaw, randomize=True, verbose=verbose)
        results.append(r)
        print(f"      试验{t+1}/{n}: {r}")
    rate = results.count("success") / n
    return rate, results


# ═══════════════════════════════════════════════════════════════
# μ 自动标定
# ═══════════════════════════════════════════════════════════════

async def auto_calibrate_mu():
    global MU_ROLLER_S_BASE, MU_ROLLER_BASE
    print("\n  [自动标定] 扫描 μ_roller_s ...")
    for mu in [0.050, 0.040, 0.030, 0.020, 0.010, 0.005]:
        _set_roller_mu(mu)
        MU_ROLLER_S_BASE = mu
        MU_ROLLER_BASE   = mu * PALLET_MU_S
        result = await run_one_trial(y=0.0, yaw=0.0, randomize=False, verbose=True)
        print(f"  μ_roller_s={mu:.3f} → {result}")
        if result == "success":
            print(f"  ★ 标定完成: MU_ROLLER_S_BASE = {mu}")
            return mu
    return None


# ═══════════════════════════════════════════════════════════════
# JSON 保存
# ═══════════════════════════════════════════════════════════════

def save_log():
    with open(SAVE_PATH, "w") as f:
        json.dump(LOG, f, indent=2, ensure_ascii=False)


# ═══════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════

async def main():
    global stage, proxy, pallet_rb_api
    global world, timeline, app
    global ROLLER_TOP_Z0, PALLET_X, PALLET_Y, PALLET_H, LOG

    stage = omni.usd.get_context().get_stage()
    proxy = stage.GetPrimAtPath(PALLET_PATH)
    if not proxy.IsValid():
        print(f"✗ 找不到 {PALLET_PATH}"); return
    pallet_rb_api = UsdPhysics.RigidBodyAPI.Get(stage, proxy.GetPath())
    if not pallet_rb_api:
        print(f"✗ {PALLET_PATH} 无 RigidBodyAPI"); return

    ROLLER_TOP_Z0 = get_scene_baseline()
    _ps = proxy.GetAttribute("xformOp:scale").Get()
    PALLET_X, PALLET_Y, PALLET_H = _ps[0], _ps[1], _ps[2]

    timeline = omni.timeline.get_timeline_interface()
    app      = omni.kit.app.get_app()

    # World 初始化（只做一次）
    from isaacsim.core.api.world import World
    if World.instance() is not None:
        World.instance().clear_instance()
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT,
                  stage_units_in_meters=1.0)
    await world.initialize_simulation_context_async()
    print("✓ World 已初始化")

    LOG = {
        "config": {
            "version"         : "no_pitch_v4",
            "MU_ROLLER_S_BASE": MU_ROLLER_S_BASE,
            "PALLET_MU_S"     : PALLET_MU_S,
            "MU_ROLLER_BASE"  : MU_ROLLER_BASE,
            "MU_RAND_RANGE"   : MU_RAND_RANGE,
            "PITCH_BASE"      : PITCH_BASE,
            "N_TRIALS"        : N_TRIALS,
            "SIM_DURATION"    : SIM_DURATION,
            "PAL_X0_ACTUAL"   : PAL_X0_ACTUAL,
            "PAL_Z0_ACTUAL"   : PAL_Z0_ACTUAL,
            "PAL_Y0_CENTER"   : PAL_Y0_CENTER,
            "PALLET_XYH"      : [float(PALLET_X), float(PALLET_Y), float(PALLET_H)],
            "ROLLER_TOP_Z0"   : float(ROLLER_TOP_Z0),
            "CONVEYOR_END_X"  : CONVEYOR_END_X,
            "SUCCESS_X"       : SUCCESS_X,
        },
        "step2": [], "step3": [],
        "step4": {"Y_grid": [], "Yaw_grid": [], "matrix": []},
        "safe_poses": [], "safe_pose_set": {},
    }

    AR = math.radians(4.0)
    print("=" * 60)
    print("参数空间搜索（无 Pitch · v4 · reset后写位姿）")
    print("=" * 60)
    print(f"  μ_roller_s : {MU_ROLLER_S_BASE}  等效μ={MU_ROLLER_BASE:.4f}")
    print(f"  tan(4°)    : {math.tan(AR):.4f}  "
          f"{'✓ 理论下滑' if MU_ROLLER_BASE < math.tan(AR) else '✗ 理论卡住'}")
    print(f"  网格规模   : {len(Y_GRID)}×{len(YAW_GRID)}×{N_TRIALS}"
          f" = {len(Y_GRID)*len(YAW_GRID)*N_TRIALS} 次")
    print(f"  预计时间   : ~{len(Y_GRID)*len(YAW_GRID)*N_TRIALS*SIM_DURATION/60:.0f} 分钟")
    print("=" * 60)

    # ── Step 1: 基准验证 verbose ──────────────────────────────
    print("\n[Step 1] 基准验证 (Y=0, Yaw=0, Pitch=4°, verbose)")
    result = await run_one_trial(y=0.0, yaw=0.0, randomize=False, verbose=True)
    print(f"  结果: {result}")

    if result != "success":
        print("\n  ⚠ 基准未通过 → 尝试自动标定 μ ...")
        calibrated = await auto_calibrate_mu()
        if calibrated is None:
            print("  ✗ 所有 μ 均失败，请检查场景配置"); save_log(); return
        result = await run_one_trial(y=0.0, yaw=0.0, randomize=False, verbose=True)
        print(f"  标定后基准: {result}")
        if result != "success":
            print("  ✗ 标定后依然失败，终止"); save_log(); return
        LOG["config"]["MU_ROLLER_S_BASE"] = MU_ROLLER_S_BASE
        LOG["config"]["MU_ROLLER_BASE"]   = MU_ROLLER_BASE

    print("  ✓ 基准通过，开始搜索\n")

    # ── Step 2: Y 偏心 ────────────────────────────────────────
    print("[Step 2] Y 偏心搜索")
    Y_CRITICAL = None
    for y in Y_SEARCH:
        print(f"\n  Y = {y*1000:.1f}mm:")
        rate, raw = await run_trials(float(y), yaw=0.0)
        status = "✓下滑" if rate >= 0.8 else ("△不稳" if rate >= 0.4 else "✗卡住")
        print(f"  → 成功率={rate:.0%} {status}")
        LOG["step2"].append({"Y_mm": round(float(y)*1000,1), "rate": round(rate,3), "raw": raw})
        save_log()
        if rate < 0.8 and Y_CRITICAL is None:
            Y_CRITICAL = float(y)
            print(f"  ★ Y_critical ≈ {y*1000:.1f}mm")
    if Y_CRITICAL is None:
        Y_CRITICAL = float(Y_SEARCH[-1])
        print(f"  ★ 全程通过，Y_critical = {Y_CRITICAL*1000:.1f}mm（上限）")

    # ── Step 3: Yaw 角 ────────────────────────────────────────
    Y_HALF = Y_CRITICAL * 0.5
    print(f"\n[Step 3] Yaw 角搜索 (Y={Y_HALF*1000:.1f}mm)")
    YAW_CRITICAL = None
    for yaw in YAW_SEARCH:
        print(f"\n  Yaw = {yaw:.1f}°:")
        rate, raw = await run_trials(Y_HALF, yaw=float(yaw))
        status = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  → 成功率={rate:.0%} {status}")
        LOG["step3"].append({"Yaw_deg": round(float(yaw),1), "rate": round(rate,3), "raw": raw})
        save_log()
        if rate < 0.8 and YAW_CRITICAL is None:
            YAW_CRITICAL = float(yaw)
            print(f"  ★ Yaw_critical ≈ {yaw:.1f}°")
    if YAW_CRITICAL is None:
        YAW_CRITICAL = float(YAW_SEARCH[-1])

    # ── Step 4: 网格 ──────────────────────────────────────────
    print(f"\n[Step 4] Y×Yaw 网格 ({len(Y_GRID)}×{len(YAW_GRID)})")
    grid = np.zeros((len(Y_GRID), len(YAW_GRID)))
    LOG["step4"]["Y_grid"]   = [round(float(y)*1000,1) for y in Y_GRID]
    LOG["step4"]["Yaw_grid"] = [round(float(w),1)      for w in YAW_GRID]

    for i, y in enumerate(Y_GRID):
        row = []
        for j, yaw in enumerate(YAW_GRID):
            print(f"\n  [{i+1}/{len(Y_GRID)},{j+1}/{len(YAW_GRID)}]"
                  f" Y={y*1000:.1f}mm Yaw={yaw:.1f}°:")
            rate, raw = await run_trials(float(y), float(yaw))
            grid[i, j] = rate
            row.append(round(rate, 3))
            if rate >= 0.80:
                LOG["safe_poses"].append({
                    "Y_mm": round(float(y)*1000,1),
                    "Yaw_deg": round(float(yaw),1),
                    "rate": round(rate,3),
                })
        LOG["step4"]["matrix"].append(row)
        save_log()
        row_str = " ".join("✓" if v >= 0.8 else ("△" if v >= 0.4 else "✗") for v in grid[i])
        print(f"\n  Y={y*1000:5.1f}mm │ {row_str}")

    # ── Step 5: Safe Pose Set ─────────────────────────────────
    print("\n" + "=" * 60)
    print("[Step 5] Safe Pose Set")
    print("=" * 60)
    restore_mu()
    safe = LOG["safe_poses"]
    if safe:
        y_vals   = [p["Y_mm"]    for p in safe]
        yaw_vals = [p["Yaw_deg"] for p in safe]
        LOG["safe_pose_set"] = {
            "position"   : {"X_m": PAL_X0_ACTUAL,
                            "Y_range_mm": [min(y_vals), max(y_vals)],
                            "Z_m": PAL_Z0_ACTUAL, "Y_center_m": PAL_Y0_CENTER},
            "orientation": {"Roll_deg": 0.0, "Pitch_deg": PITCH_BASE,
                            "Yaw_range_deg": [min(yaw_vals), max(yaw_vals)]},
            "friction"   : {"mu_roller_s_base": MU_ROLLER_S_BASE,
                            "mu_pallet_s": PALLET_MU_S,
                            "mu_eff_base": MU_ROLLER_BASE,
                            "mu_eff_range": [round(MU_ROLLER_BASE*(1-MU_RAND_RANGE),4),
                                             round(MU_ROLLER_BASE*(1+MU_RAND_RANGE),4)]},
            "critical"   : {"Y_critical_mm": round(Y_CRITICAL*1000,1),
                            "Yaw_critical_deg": round(YAW_CRITICAL,1)},
            "stats"      : {"n_safe": len(safe),
                            "total_grid": len(Y_GRID)*len(YAW_GRID),
                            "safe_ratio": round(len(safe)/(len(Y_GRID)*len(YAW_GRID)),3)},
        }
        save_log()
        print(f"  Y 安全范围   : {min(y_vals):.1f} ~ {max(y_vals):.1f} mm")
        print(f"  Yaw 安全范围 : {min(yaw_vals):.1f} ~ {max(yaw_vals):.1f} °")
        print(f"  安全格数     : {len(safe)}/{len(Y_GRID)*len(YAW_GRID)}"
              f" ({len(safe)/(len(Y_GRID)*len(YAW_GRID))*100:.0f}%)")
        print(f"\n  → 已保存: {SAVE_PATH}")
    else:
        print("  ⚠ 未找到安全区域")
        save_log()

    print("\n✅ 搜索完成")


asyncio.ensure_future(main())
