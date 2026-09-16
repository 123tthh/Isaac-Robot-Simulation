"""
search_dataspace_no_pitch.py  (Isaac Sim 5.1 · Script Editor 专用)
──────────────────────────────────────────────────────────────────
参考文档：
  • python_scripting/robots_simulation.html     ← World/Articulation 异步模式
  • python_scripting/util_snippets.html         ← next_update_async 推帧
  • replicator_tutorials/tutorial_replicator_isaac_snippets.html
      (simulation_get_data.py 的 Script Editor 版) ← 每帧从 PhysX 读速度
  • reference_material/sim_performance_optimization_handbook.html

相对旧版的修正：
  1. 整个流程改为 async，用 await next_update_async() 推进每一帧
  2. 用 World 统一管生命周期，只在开头 initialize 一次
  3. 用 SingleRigidPrim 从 PhysX 读位姿/速度（USD 属性在仿真期间不同步）
  4. 每次试验：timeline.stop → 写 USD → world.reset_async → 循环推帧 → 读状态
  5. 每格/每步跑完就增量保存 JSON，防止中途崩溃丢数据
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

# ═════════════════════════════════════════════════════════════
# ★ 可调参数
# ═════════════════════════════════════════════════════════════

# ── 摩擦 ──────────────────────────────────────────────────────
MU_ROLLER_S_BASE = 0.060
PALLET_MU_S      = 0.400
MU_ROLLER_BASE   = MU_ROLLER_S_BASE * PALLET_MU_S
MU_RAND_RANGE    = 0.30

# ── 搜索空间 ──────────────────────────────────────────────────
Y_SEARCH   = np.linspace(0.000, 0.040, 9)
YAW_SEARCH = np.linspace(0.0,   8.0,   9)
Y_GRID     = np.linspace(0.000, 0.040, 7)
YAW_GRID   = np.linspace(0.0,   8.0,   7)

# ── 仿真参数 ──────────────────────────────────────────────────
N_TRIALS     = 5
SIM_DURATION = 5.0
PHYSICS_DT   = 1.0 / 60.0
STUCK_SPEED  = 0.005

# ── 位置 ─────────────────────────────────────────────────────
PAL_X0_ACTUAL  = 0.045
PAL_Z0_ACTUAL  = 1.26751
PAL_Y0_CENTER  = 0.00253
PITCH_BASE     = 4.0

SAVE_PATH = "/tmp/safe_pose_set_no_pitch.json"
PALLET_PATH = "/World/Pallet/CollisionProxy"

# ═════════════════════════════════════════════════════════════
# 全局状态（启动时填）
# ═════════════════════════════════════════════════════════════
stage        = None
proxy        = None
pallet_prim  = None
pallet_rb_api = None
world        = None
timeline     = None
app          = None

ROLLER_TOP_Z0  = None
PALLET_X       = None
PALLET_Y       = None
PALLET_H       = None
CONVEYOR_END_X = 0.500
SUCCESS_X      = CONVEYOR_END_X * 0.9

LOG = None   # 记录体，主流程里初始化


# ═════════════════════════════════════════════════════════════
# USD / 物理工具函数
# ═════════════════════════════════════════════════════════════
def get_scene_baseline():
    r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
    cg  = stage.GetPrimAtPath("/World/ConveyorGroup")
    if not r00.IsValid():
        raise RuntimeError("Roller_00_L 不存在，检查路径")
    r_z  = r00.GetAttribute("xformOp:translate").Get()[2]
    cg_z = cg.GetAttribute("xformOp:translate").Get()[2] if cg.IsValid() else 0.0
    return r_z + cg_z + 0.010


def set_pallet_pose_usd(y, yaw_deg, pitch_deg=None):
    """Stop 状态下写入 USD，play 后 reset_async 才会被 PhysX 读取。"""
    if pitch_deg is None:
        pitch_deg = PITCH_BASE

    xf = UsdGeom.XformCommonAPI(proxy)
    xf.SetTranslate(Gf.Vec3d(
        PAL_X0_ACTUAL,
        PAL_Y0_CENTER + y,
        PAL_Z0_ACTUAL,
    ))
    xf.SetRotate(
        Gf.Vec3f(0.0, float(pitch_deg), float(yaw_deg)),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )

    # 速度也要在 Stop 状态下清零，play 时才会生效
    if pallet_rb_api:
        pallet_rb_api.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        pallet_rb_api.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))


def randomize_mu():
    mat = stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return
    mu_eff_new = MU_ROLLER_BASE * (
        1.0 + np.random.uniform(-MU_RAND_RANGE, MU_RAND_RANGE)
    )
    mu_eff_new = float(np.clip(mu_eff_new, 0.005, 0.065))
    mu_s = mu_eff_new / PALLET_MU_S
    mu_d = mu_s * 0.7
    m = UsdPhysics.MaterialAPI.Get(stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(mu_s)
        m.GetDynamicFrictionAttr().Set(mu_d)


def restore_mu():
    mat = stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return
    m = UsdPhysics.MaterialAPI.Get(stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(MU_ROLLER_S_BASE)
        m.GetDynamicFrictionAttr().Set(MU_ROLLER_S_BASE * 0.7)


def classify(pos, vel):
    """基于 PhysX 读来的 pos/vel 分类。"""
    if pos is None:
        return "error"
    x = float(pos[0])
    z = float(pos[2])
    speed = float((vel[0]**2 + vel[1]**2 + vel[2]**2) ** 0.5)
    if x >= SUCCESS_X:
        return "success"
    elif z < PAL_Z0_ACTUAL - 0.15:
        return "fallen"
    elif speed < STUCK_SPEED:
        return "stuck"
    else:
        return "timeout"


# ═════════════════════════════════════════════════════════════
# 单次试验 — 异步，唯一正确的推帧方式
# ═════════════════════════════════════════════════════════════
async def run_one_trial(y, yaw, randomize=True):
    # A. 停 timeline，才能改 USD
    if timeline.is_playing():
        timeline.stop()

    # B. 改摩擦 + 位姿
    if randomize:
        randomize_mu()
    else:
        restore_mu()
    set_pallet_pose_usd(y, yaw_deg=yaw, pitch_deg=PITCH_BASE)

    # C. reset_async：stop→set_initial_state→play→走 1 帧
    await world.reset_async()

    # D. 用 next_update_async 连续推帧（ = 物理 + 渲染 + USD 同步）
    n_steps = int(SIM_DURATION / PHYSICS_DT)
    for _ in range(n_steps):
        await app.next_update_async()

    # E. 从 PhysX tensor 直接读最终状态（USD 属性靠不住）
    pos, _quat = pallet_prim.get_world_pose()
    lin_vel    = pallet_prim.get_linear_velocity()

    # F. 停 timeline 准备下一次
    timeline.stop()

    return classify(pos, lin_vel)


async def run_trials(y, yaw, n=N_TRIALS):
    results = []
    for t in range(n):
        r = await run_one_trial(y, yaw, randomize=True)
        results.append(r)
        print(f"      试验{t+1}/{n}: {r}")
    rate = results.count("success") / n
    return rate, results


# ═════════════════════════════════════════════════════════════
# JSON 增量保存
# ═════════════════════════════════════════════════════════════
def save_log():
    with open(SAVE_PATH, "w") as f:
        json.dump(LOG, f, indent=2, ensure_ascii=False)


# ═════════════════════════════════════════════════════════════
# 主搜索流程（全异步）
# ═════════════════════════════════════════════════════════════
async def main():
    global stage, proxy, pallet_prim, pallet_rb_api
    global world, timeline, app
    global ROLLER_TOP_Z0, PALLET_X, PALLET_Y, PALLET_H, LOG

    # ── 0. Stage / Prim 就绪 ──────────────────────────────────
    stage = omni.usd.get_context().get_stage()
    proxy = stage.GetPrimAtPath(PALLET_PATH)
    if not proxy.IsValid():
        print(f"✗ 找不到 {PALLET_PATH}"); return
    pallet_rb_api = UsdPhysics.RigidBodyAPI.Get(stage, proxy.GetPath())

    ROLLER_TOP_Z0 = get_scene_baseline()
    _ps = proxy.GetAttribute("xformOp:scale").Get()
    PALLET_X, PALLET_Y, PALLET_H = _ps[0], _ps[1], _ps[2]

    timeline = omni.timeline.get_timeline_interface()
    app      = omni.kit.app.get_app()

    # ── 1. 建 World 并初始化（只做一次）───────────────────────
    from isaacsim.core.api.world import World
    if World.instance() is not None:
        World.instance().clear_instance()
    world = World(
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        stage_units_in_meters=1.0,
    )
    await world.initialize_simulation_context_async()
    print("✓ World 已初始化")

    # ── 2. 包装托盘为 SingleRigidPrim（才能读 PhysX tensor）──
    try:
        from isaacsim.core.prims import SingleRigidPrim
        pallet_prim = SingleRigidPrim(prim_path=PALLET_PATH, name="pallet")
    except ImportError:
        from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
        pallet_prim = SingleRigidPrim(prim_path=PALLET_PATH, name="pallet")
    print(f"✓ SingleRigidPrim 已包装: {PALLET_PATH}")

    # ── 3. LOG 容器 ───────────────────────────────────────────
    LOG = {
        "config": {
            "version"         : "no_pitch_async_v2",
            "MU_ROLLER_S_BASE": MU_ROLLER_S_BASE,
            "PALLET_MU_S"     : PALLET_MU_S,
            "MU_ROLLER_BASE"  : MU_ROLLER_BASE,
            "MU_RAND_RANGE"   : MU_RAND_RANGE,
            "PITCH_BASE"      : PITCH_BASE,
            "PITCH_RANDOMIZED": False,
            "N_TRIALS"        : N_TRIALS,
            "SIM_DURATION"    : SIM_DURATION,
            "PHYSICS_DT"      : PHYSICS_DT,
            "PAL_X0_ACTUAL"   : PAL_X0_ACTUAL,
            "PAL_Z0_ACTUAL"   : PAL_Z0_ACTUAL,
            "PAL_Y0_CENTER"   : PAL_Y0_CENTER,
            "PALLET_XYH"      : [float(PALLET_X), float(PALLET_Y), float(PALLET_H)],
            "ROLLER_TOP_Z0"   : float(ROLLER_TOP_Z0),
            "CONVEYOR_END_X"  : CONVEYOR_END_X,
            "SUCCESS_X"       : SUCCESS_X,
        },
        "step2"        : [],
        "step3"        : [],
        "step4"        : {"Y_grid": [], "Yaw_grid": [], "matrix": []},
        "safe_poses"   : [],
        "safe_pose_set": {},
    }

    AR = math.radians(4.0)
    print("=" * 60)
    print("参数空间搜索（无 Pitch 随机化 · async v2）")
    print("=" * 60)
    print(f"  μ_roller_s基准  : {MU_ROLLER_S_BASE}")
    print(f"  等效μ基准       : {MU_ROLLER_BASE:.4f}")
    print(f"  域随机化        : μ ±{MU_RAND_RANGE*100:.0f}%"
          f" → [{MU_ROLLER_BASE*(1-MU_RAND_RANGE):.4f},"
          f" {MU_ROLLER_BASE*(1+MU_RAND_RANGE):.4f}]")
    print(f"  Pitch           : {PITCH_BASE}°（固定）")
    print(f"  tan(4°)         : {math.tan(AR):.4f}")
    print(f"  托盘尺寸        : X={PALLET_X*1000:.1f}"
          f" Y={PALLET_Y*1000:.1f} H={PALLET_H*1000:.1f}mm")
    print(f"  成功判定 X      : {SUCCESS_X:.3f}m")
    print(f"  网格规模        : {len(Y_GRID)}×{len(YAW_GRID)}×{N_TRIALS}"
          f" = {len(Y_GRID)*len(YAW_GRID)*N_TRIALS} 次仿真")
    print(f"  预计时间        : ~{len(Y_GRID)*len(YAW_GRID)*N_TRIALS*SIM_DURATION/60:.0f} 分钟")
    print("=" * 60)

    # ── Step 1: 基准 ─────────────────────────────────────────
    print("\n[Step 1] 基准验证 (Y=0, Yaw=0, Pitch=4°, μ基准)")
    result = await run_one_trial(y=0.0, yaw=0.0, randomize=False)
    print(f"  结果: {result}")
    if result != "success":
        print("\n  ⚠ 基准未通过！")
        print(f"  原因1: μ太大 → 降低 MU_ROLLER_S_BASE (当前 {MU_ROLLER_S_BASE})")
        print("  原因2: 托盘位置偏差 → 检查 PAL_Z0_ACTUAL")
        save_log()
        return
    print("  ✓ 基准通过")

    # ── Step 2: Y 偏心 ──────────────────────────────────────
    print(f"\n[Step 2] Y 偏心搜索 (Yaw=0 固定，μ 随机)")
    print(f"  Y: {Y_SEARCH[0]*1000:.0f}~{Y_SEARCH[-1]*1000:.0f}mm"
          f" | {N_TRIALS}次/点")

    Y_CRITICAL = None
    for y in Y_SEARCH:
        print(f"\n  Y = {y*1000:.1f}mm:")
        rate, raw = await run_trials(float(y), yaw=0.0)
        status = "✓下滑" if rate >= 0.8 else ("△不稳" if rate >= 0.4 else "✗卡住")
        print(f"  → 成功率={rate:.0%} {status}")
        LOG["step2"].append({
            "Y_mm": round(float(y)*1000, 1),
            "rate": round(rate, 3),
            "raw" : raw,
        })
        save_log()
        if rate < 0.8 and Y_CRITICAL is None:
            Y_CRITICAL = float(y)
            print(f"  ★ Y_critical ≈ {y*1000:.1f}mm")
    if Y_CRITICAL is None:
        Y_CRITICAL = float(Y_SEARCH[-1])
        print(f"  ★ 全程通过，Y_critical = {Y_CRITICAL*1000:.1f}mm（上限）")

    # ── Step 3: Yaw 角 ──────────────────────────────────────
    Y_HALF = Y_CRITICAL * 0.5
    print(f"\n[Step 3] Yaw 角搜索 (Y={Y_HALF*1000:.1f}mm 固定)")

    YAW_CRITICAL = None
    for yaw in YAW_SEARCH:
        print(f"\n  Yaw = {yaw:.1f}°:")
        rate, raw = await run_trials(Y_HALF, yaw=float(yaw))
        status = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  → 成功率={rate:.0%} {status}")
        LOG["step3"].append({
            "Yaw_deg": round(float(yaw), 1),
            "rate"   : round(rate, 3),
            "raw"    : raw,
        })
        save_log()
        if rate < 0.8 and YAW_CRITICAL is None:
            YAW_CRITICAL = float(yaw)
            print(f"  ★ Yaw_critical ≈ {yaw:.1f}°")
    if YAW_CRITICAL is None:
        YAW_CRITICAL = float(YAW_SEARCH[-1])

    # ── Step 4: Y × Yaw 网格 ────────────────────────────────
    print(f"\n[Step 4] Y×Yaw 网格  ({len(Y_GRID)}×{len(YAW_GRID)})")

    grid = np.zeros((len(Y_GRID), len(YAW_GRID)))
    LOG["step4"]["Y_grid"]   = [round(float(y)*1000, 1) for y in Y_GRID]
    LOG["step4"]["Yaw_grid"] = [round(float(w), 1)      for w in YAW_GRID]

    for i, y in enumerate(Y_GRID):
        row = []
        for j, yaw in enumerate(YAW_GRID):
            print(f"\n  [{i+1}/{len(Y_GRID)}, {j+1}/{len(YAW_GRID)}]"
                  f" Y={y*1000:.1f}mm Yaw={yaw:.1f}°:")
            rate, raw = await run_trials(float(y), float(yaw))
            grid[i, j] = rate
            row.append(round(rate, 3))
            if rate >= 0.80:
                LOG["safe_poses"].append({
                    "Y_mm"   : round(float(y)*1000, 1),
                    "Yaw_deg": round(float(yaw), 1),
                    "rate"   : round(rate, 3),
                })
        LOG["step4"]["matrix"].append(row)
        save_log()
        row_str = " ".join(
            "✓" if v >= 0.8 else ("△" if v >= 0.4 else "✗")
            for v in grid[i]
        )
        print(f"\n  Y={y*1000:5.1f}mm │ {row_str}")

    # ── Step 5: Safe Pose Set ───────────────────────────────
    print("\n" + "=" * 60)
    print("[Step 5] Safe Pose Set")
    print("=" * 60)
    restore_mu()

    safe = LOG["safe_poses"]
    if safe:
        y_vals   = [p["Y_mm"]    for p in safe]
        yaw_vals = [p["Yaw_deg"] for p in safe]
        LOG["safe_pose_set"] = {
            "position": {
                "X_m"        : PAL_X0_ACTUAL,
                "Y_range_mm" : [min(y_vals), max(y_vals)],
                "Z_m"        : PAL_Z0_ACTUAL,
                "Y_center_m" : PAL_Y0_CENTER,
            },
            "orientation": {
                "Roll_deg"     : 0.0,
                "Pitch_deg"    : PITCH_BASE,
                "Yaw_range_deg": [min(yaw_vals), max(yaw_vals)],
            },
            "friction": {
                "mu_roller_s_base": MU_ROLLER_S_BASE,
                "mu_pallet_s"     : PALLET_MU_S,
                "mu_eff_base"     : MU_ROLLER_BASE,
                "mu_eff_range"    : [
                    round(MU_ROLLER_BASE*(1-MU_RAND_RANGE), 4),
                    round(MU_ROLLER_BASE*(1+MU_RAND_RANGE), 4),
                ],
            },
            "critical": {
                "Y_critical_mm"   : round(Y_CRITICAL*1000, 1),
                "Yaw_critical_deg": round(YAW_CRITICAL, 1),
            },
            "stats": {
                "n_safe"    : len(safe),
                "total_grid": len(Y_GRID) * len(YAW_GRID),
                "safe_ratio": round(len(safe)/(len(Y_GRID)*len(YAW_GRID)), 3),
            },
        }
        save_log()
        print(f"""
  Y 安全范围   : {min(y_vals):.1f} ~ {max(y_vals):.1f} mm
  Yaw 安全范围 : {min(yaw_vals):.1f} ~ {max(yaw_vals):.1f} °
  Y_critical   : {Y_CRITICAL*1000:.1f} mm
  Yaw_critical : {YAW_CRITICAL:.1f} °
  安全格数     : {len(safe)} / {len(Y_GRID)*len(YAW_GRID)}"""
              f" ({len(safe)/(len(Y_GRID)*len(YAW_GRID))*100:.0f}%)\n"
              f"\n  → 已保存: {SAVE_PATH}\n"
              f"  → 此区间作为 RL 训练目标位姿")
    else:
        print("  ⚠ 未找到安全区域")
        print(f"  建议: 降低 MU_ROLLER_S_BASE (当前 {MU_ROLLER_S_BASE})")
        save_log()

    print("\n✅ 搜索完成（无 Pitch · async v2）")


# 启动（Script Editor 里这一句放在最外层）
asyncio.ensure_future(main())
