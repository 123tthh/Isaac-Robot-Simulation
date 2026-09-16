"""
search_dataspace_no_pitch_v5.py  (Isaac Sim 5.1 · Script Editor)
═══════════════════════════════════════════════════════════════════
v5 修改点：
  1. 判定改为：速度 < 0.005 m/s 持续 ≥ STUCK_DURATION 秒 → stuck
     （而不是只看最后一帧速度）
  2. 删除 fallen 判定（轨道有挡板，不会掉落）
  3. 输出路径改为 projects/physics_parameters/results/
  4. 无论成功/失败，所有 trial 结果都写入 JSON，结束后可自行筛选
  5. 卡住时提前终止当前 trial，直接进入下一次（不等满 SIM_DURATION）
  6. 基准验证失败时打印详细诊断，不自动标定（避免额外复杂度）
  7. 每次 trial 都在 verbose 模式下运行（每秒打一次位置），方便观察
═══════════════════════════════════════════════════════════════════
"""

import os

from pathlib import Path as _ProjectPath

def _physics_project_root():
    candidates = [os.environ.get("ISAAC_OCS_PROJECT_ROOT"), globals().get("__file__"), str(_ProjectPath.cwd())]
    for candidate in candidates:
        if not candidate:
            continue
        start = _ProjectPath(candidate).expanduser().resolve()
        if start.is_file():
            start = start.parent
        for parent in (start, *start.parents):
            if (parent / "project_manifest.yaml").is_file():
                return parent
    raise RuntimeError("Set ISAAC_OCS_PROJECT_ROOT to the project directory before running in Script Editor")

_PHYSICS_ROOT = _physics_project_root() / "projects" / "physics_parameters"


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

N_TRIALS      = 5
SIM_DURATION  = 8.0          # 最长等待秒数
PHYSICS_DT    = 1.0 / 60.0
STUCK_SPEED   = 0.005        # m/s，低于此视为静止
STUCK_DURATION = 3.0         # 静止持续超过此秒数 → 判定卡住并提前终止
POSE_SETTLE_STEPS = 5        # reset后空走几帧让物理稳定

PAL_X0_ACTUAL = 0.045
PAL_Z0_ACTUAL = 1.26751
PAL_Y0_CENTER = 0.00253
PITCH_BASE    = 4.0

# 输出路径
RESULT_DIR = str(_PHYSICS_ROOT / "results")
SAVE_PATH  = os.path.join(RESULT_DIR, "safe_pose_set_no_pitch.json")

PALLET_PATH    = "/World/Pallet/CollisionProxy"
CONVEYOR_END_X = 0.500
SUCCESS_X      = CONVEYOR_END_X * 0.9   # 0.45m

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

PALLET_X = PALLET_Y = PALLET_H = None
ROLLER_TOP_Z0 = None
LOG = None


# ═══════════════════════════════════════════════════════════════
# 工具
# ═══════════════════════════════════════════════════════════════

def get_scene_baseline():
    r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
    cg  = stage.GetPrimAtPath("/World/ConveyorGroup")
    if not r00.IsValid():
        raise RuntimeError("Roller_00_L 不存在，检查路径")
    r_z  = r00.GetAttribute("xformOp:translate").Get()[2]
    cg_z = cg.GetAttribute("xformOp:translate").Get()[2] if cg.IsValid() else 0.0
    return r_z + cg_z + 0.010


def set_pallet_pose_usd(y, yaw_deg, pitch_deg=None):
    """reset_async 之后调用，写入位姿，PhysX 下帧读取。"""
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
        print("  ⚠ /World/Mat/Roller 不存在，摩擦未设置")
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


# ═══════════════════════════════════════════════════════════════
# 单次试验：基于持续时间的卡住判定，提前终止
# ═══════════════════════════════════════════════════════════════

async def run_one_trial(y, yaw, randomize=True, trial_id=""):
    global pallet_prim

    # 1. 停 timeline
    if timeline.is_playing():
        timeline.stop()

    # 2. 摩擦设置
    if randomize:
        randomize_mu()
    else:
        restore_mu()

    # 3. reset_async（恢复场景初始状态 + 自动 play）
    await world.reset_async()

    # 4. ★ reset 之后写位姿（reset 会覆盖 reset 前的写入）
    set_pallet_pose_usd(y, yaw_deg=yaw, pitch_deg=PITCH_BASE)

    # 5. reset 后重新绑定 SingleRigidPrim
    try:
        from isaacsim.core.prims import SingleRigidPrim
    except ImportError:
        from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
    pallet_prim = SingleRigidPrim(prim_path=PALLET_PATH, name="pallet_trial")

    # 6. Settle 帧
    for _ in range(POSE_SETTLE_STEPS):
        await app.next_update_async()

    # 验证位置是否写入
    pos_check, _ = pallet_prim.get_world_pose()
    x_start = float(pos_check[0])
    print(f"      [{trial_id}] 起始X={x_start:.4f}m "
          f"(期望={PAL_X0_ACTUAL})", end="")
    if abs(x_start - PAL_X0_ACTUAL) > 0.01:
        print(" ⚠ 位置偏差较大！")
    else:
        print(" ✓")

    # 7. 逐帧推进，实时判定
    n_steps       = int(SIM_DURATION / PHYSICS_DT)
    stuck_counter = 0                         # 连续静止帧计数
    stuck_frames  = int(STUCK_DURATION / PHYSICS_DT)  # 判定卡住所需帧数
    result        = "timeout"
    final_x       = x_start
    final_speed   = 0.0
    log_interval  = 60  # 每秒打印

    for step in range(n_steps):
        await app.next_update_async()

        pos, _ = pallet_prim.get_world_pose()
        vel    = pallet_prim.get_linear_velocity()
        x      = float(pos[0])
        speed  = float((vel[0]**2 + vel[1]**2 + vel[2]**2) ** 0.5)
        final_x     = x
        final_speed = speed

        # ★ 卡住判定：速度持续低于阈值
        if speed < STUCK_SPEED:
            stuck_counter += 1
        else:
            stuck_counter = 0   # 重新开始计数（有运动就清零）

        # 每秒打一次
        if (step + 1) % log_interval == 0:
            t = (step + 1) * PHYSICS_DT
            stuck_sec = stuck_counter * PHYSICS_DT
            print(f"      {t:.1f}s | X={x:.4f}m | V={speed:.4f}m/s"
                  f" | 静止={stuck_sec:.1f}s")

        # 成功：到达终点
        if x >= SUCCESS_X:
            result = "success"
            break

        # 失败：持续静止超过 STUCK_DURATION 秒
        if stuck_counter >= stuck_frames:
            result = "stuck"
            break

    else:
        # 循环正常结束（跑满 SIM_DURATION 还没到终点）
        if final_x >= SUCCESS_X:
            result = "success"
        elif final_speed < STUCK_SPEED:
            result = "stuck"
        else:
            result = "timeout"

    # 8. 停止
    timeline.stop()

    print(f"      → {result} | 最终X={final_x:.4f}m V={final_speed:.4f}m/s")
    return result, final_x, final_speed


# ═══════════════════════════════════════════════════════════════
# 多次试验：所有结果都记录
# ═══════════════════════════════════════════════════════════════

async def run_trials(y, yaw, n=N_TRIALS):
    records = []
    for t in range(n):
        label = f"trial{t+1}/{n}"
        result, fx, fv = await run_one_trial(y, yaw, randomize=True, trial_id=label)
        records.append({
            "result"  : result,
            "final_x" : round(fx, 5),
            "final_v" : round(fv, 5),
        })
    success_rate = sum(1 for r in records if r["result"] == "success") / n
    return success_rate, records


# ═══════════════════════════════════════════════════════════════
# JSON 保存（增量）
# ═══════════════════════════════════════════════════════════════

def save_log():
    os.makedirs(RESULT_DIR, exist_ok=True)
    with open(SAVE_PATH, "w") as f:
        json.dump(LOG, f, indent=2, ensure_ascii=False)
    print(f"      → 已保存: {SAVE_PATH}")


# ═══════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════

async def main():
    global stage, proxy, pallet_rb_api
    global world, timeline, app
    global PALLET_X, PALLET_Y, PALLET_H, ROLLER_TOP_Z0, LOG

    # ── 0. 基础检查 ────────────────────────────────────────────
    os.makedirs(RESULT_DIR, exist_ok=True)
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

    # ── 1. World 初始化（只做一次）────────────────────────────
    from isaacsim.core.api.world import World
    if World.instance() is not None:
        World.instance().clear_instance()
    world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT,
                  stage_units_in_meters=1.0)
    await world.initialize_simulation_context_async()
    print("✓ World 已初始化")
    print(f"✓ 结果保存至: {SAVE_PATH}")

    # ── 2. 初始化 LOG ──────────────────────────────────────────
    AR = math.radians(4.0)
    LOG = {
        "config": {
            "version"           : "no_pitch_v5",
            "MU_ROLLER_S_BASE"  : MU_ROLLER_S_BASE,
            "PALLET_MU_S"       : PALLET_MU_S,
            "MU_ROLLER_BASE"    : MU_ROLLER_BASE,
            "MU_RAND_RANGE"     : MU_RAND_RANGE,
            "PITCH_BASE"        : PITCH_BASE,
            "N_TRIALS"          : N_TRIALS,
            "SIM_DURATION"      : SIM_DURATION,
            "STUCK_SPEED"       : STUCK_SPEED,
            "STUCK_DURATION"    : STUCK_DURATION,
            "PAL_X0_ACTUAL"     : PAL_X0_ACTUAL,
            "PAL_Z0_ACTUAL"     : PAL_Z0_ACTUAL,
            "PAL_Y0_CENTER"     : PAL_Y0_CENTER,
            "PALLET_XYH"        : [float(PALLET_X), float(PALLET_Y), float(PALLET_H)],
            "ROLLER_TOP_Z0"     : float(ROLLER_TOP_Z0),
            "CONVEYOR_END_X"    : CONVEYOR_END_X,
            "SUCCESS_X"         : SUCCESS_X,
            "RESULT_DIR"        : RESULT_DIR,
        },
        # 所有原始 trial 数据（无论成功/失败都记录）
        "all_trials": [],
        # 按搜索步骤汇总的成功率
        "step2": [],
        "step3": [],
        "step4": {"Y_grid": [], "Yaw_grid": [], "matrix": [], "matrix_raw": []},
        # 成功率 >= 0.8 的格子
        "safe_poses"   : [],
        "safe_pose_set": {},
    }

    print("=" * 62)
    print("参数空间搜索（无 Pitch · v5）")
    print("=" * 62)
    print(f"  等效μ={MU_ROLLER_BASE:.4f}  tan(4°)={math.tan(AR):.4f}"
          f"  {'✓理论下滑' if MU_ROLLER_BASE < math.tan(AR) else '✗理论卡住'}")
    print(f"  卡住判定: |V| < {STUCK_SPEED} m/s 持续 ≥ {STUCK_DURATION}s → 提前终止")
    print(f"  网格: {len(Y_GRID)}×{len(YAW_GRID)}×{N_TRIALS}"
          f" = {len(Y_GRID)*len(YAW_GRID)*N_TRIALS} 次试验")
    print(f"  预计时间: ~{len(Y_GRID)*len(YAW_GRID)*N_TRIALS*SIM_DURATION/60:.0f} 分钟（最长）")
    print("=" * 62)

    # ── Step 1: 基准验证 ──────────────────────────────────────
    print("\n[Step 1] 基准验证 (Y=0, Yaw=0, Pitch=4°, μ基准, 无随机)")
    result, fx, fv = await run_one_trial(
        y=0.0, yaw=0.0, randomize=False, trial_id="基准")
    LOG["all_trials"].append({
        "step": "baseline", "Y_mm": 0.0, "Yaw_deg": 0.0,
        "result": result, "final_x": round(fx,5), "final_v": round(fv,5),
    })
    save_log()
    print(f"\n  基准结果: {result}")

    if result != "success":
        print("\n  ⚠ 基准未通过，诊断信息：")
        print(f"    μ_roller_s = {MU_ROLLER_S_BASE}")
        print(f"    μ_eff = {MU_ROLLER_BASE:.4f}（Roller μs × Pallet μs）")
        print(f"    tan(4°) = {math.tan(AR):.4f}")
        print(f"    理论条件: μ_eff < tan(4°)  →  "
              f"{'满足' if MU_ROLLER_BASE < math.tan(AR) else '不满足，需降低 MU_ROLLER_S_BASE'}")
        print(f"    最终 X = {fx:.4f}m（初始={PAL_X0_ACTUAL}，终点={SUCCESS_X}）")
        if abs(fx - PAL_X0_ACTUAL) < 0.005:
            print("    → 托盘几乎没动，可能是摩擦过大或位置未正确写入PhysX")
            print("    → 建议：把 MU_ROLLER_S_BASE 改为 0.030 再跑")
        elif fx > PAL_X0_ACTUAL:
            print(f"    → 托盘移动了 {(fx-PAL_X0_ACTUAL)*1000:.1f}mm 但未到终点")
            print("    → 建议：延长 SIM_DURATION 或确认坡度方向正确")
        print("\n  脚本终止，修正参数后重新运行")
        return

    print("  ✓ 基准通过，开始搜索\n")

    # ── Step 2: Y 偏心搜索 ────────────────────────────────────
    print("[Step 2] Y 偏心搜索（Yaw=0 固定）")
    print(f"  Y 范围: {Y_SEARCH[0]*1000:.0f}~{Y_SEARCH[-1]*1000:.0f}mm"
          f"，{N_TRIALS} 次/点\n")

    Y_CRITICAL = None
    for y in Y_SEARCH:
        print(f"  ── Y = {y*1000:.1f}mm ──")
        rate, records = await run_trials(float(y), yaw=0.0)

        # 记录所有 trial
        for i, rec in enumerate(records):
            LOG["all_trials"].append({
                "step": "step2", "Y_mm": round(float(y)*1000,1), "Yaw_deg": 0.0,
                "trial": i+1, **rec,
            })

        status = "✓下滑" if rate >= 0.8 else ("△不稳" if rate >= 0.4 else "✗卡住")
        print(f"  → 成功率={rate:.0%} {status}")
        LOG["step2"].append({
            "Y_mm": round(float(y)*1000,1),
            "rate": round(rate,3),
            "records": records,
        })
        save_log()

        if rate < 0.8 and Y_CRITICAL is None:
            Y_CRITICAL = float(y)
            print(f"  ★ Y_critical ≈ {y*1000:.1f}mm")

    if Y_CRITICAL is None:
        Y_CRITICAL = float(Y_SEARCH[-1])
        print(f"  ★ 全程通过，Y_critical = {Y_CRITICAL*1000:.1f}mm（上限）")

    # ── Step 3: Yaw 角搜索 ────────────────────────────────────
    Y_HALF = Y_CRITICAL * 0.5
    print(f"\n[Step 3] Yaw 角搜索（Y={Y_HALF*1000:.1f}mm 固定）\n")

    YAW_CRITICAL = None
    for yaw in YAW_SEARCH:
        print(f"  ── Yaw = {yaw:.1f}° ──")
        rate, records = await run_trials(Y_HALF, yaw=float(yaw))

        for i, rec in enumerate(records):
            LOG["all_trials"].append({
                "step": "step3", "Y_mm": round(Y_HALF*1000,1),
                "Yaw_deg": round(float(yaw),1), "trial": i+1, **rec,
            })

        status = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  → 成功率={rate:.0%} {status}")
        LOG["step3"].append({
            "Yaw_deg": round(float(yaw),1),
            "rate"   : round(rate,3),
            "records": records,
        })
        save_log()

        if rate < 0.8 and YAW_CRITICAL is None:
            YAW_CRITICAL = float(yaw)
            print(f"  ★ Yaw_critical ≈ {yaw:.1f}°")

    if YAW_CRITICAL is None:
        YAW_CRITICAL = float(YAW_SEARCH[-1])

    # ── Step 4: Y × Yaw 网格 ─────────────────────────────────
    print(f"\n[Step 4] Y×Yaw 网格（{len(Y_GRID)}×{len(YAW_GRID)}）\n")

    grid_rate = np.zeros((len(Y_GRID), len(YAW_GRID)))
    LOG["step4"]["Y_grid"]   = [round(float(y)*1000,1) for y in Y_GRID]
    LOG["step4"]["Yaw_grid"] = [round(float(w),1)      for w in YAW_GRID]

    for i, y in enumerate(Y_GRID):
        row_rate = []
        row_raw  = []
        for j, yaw in enumerate(YAW_GRID):
            print(f"  ── [{i+1}/{len(Y_GRID)}, {j+1}/{len(YAW_GRID)}]"
                  f" Y={y*1000:.1f}mm  Yaw={yaw:.1f}° ──")
            rate, records = await run_trials(float(y), float(yaw))
            grid_rate[i, j] = rate
            row_rate.append(round(rate, 3))
            row_raw.append(records)

            for k, rec in enumerate(records):
                LOG["all_trials"].append({
                    "step"   : "step4",
                    "grid_i" : i, "grid_j": j,
                    "Y_mm"   : round(float(y)*1000,1),
                    "Yaw_deg": round(float(yaw),1),
                    "trial"  : k+1, **rec,
                })

            if rate >= 0.80:
                LOG["safe_poses"].append({
                    "Y_mm"   : round(float(y)*1000,1),
                    "Yaw_deg": round(float(yaw),1),
                    "rate"   : round(rate,3),
                })
            print(f"  → 成功率={rate:.0%}\n")

        LOG["step4"]["matrix"].append(row_rate)
        LOG["step4"]["matrix_raw"].append(row_raw)
        save_log()

        row_str = " ".join(
            "✓" if v >= 0.8 else ("△" if v >= 0.4 else "✗")
            for v in grid_rate[i]
        )
        print(f"  Y={y*1000:5.1f}mm │ {row_str}\n")

    # ── Step 5: 汇总 Safe Pose Set ───────────────────────────
    print("=" * 62)
    print("[Step 5] 汇总 Safe Pose Set")
    print("=" * 62)
    restore_mu()

    safe = LOG["safe_poses"]
    if safe:
        y_vals   = [p["Y_mm"]    for p in safe]
        yaw_vals = [p["Yaw_deg"] for p in safe]
        LOG["safe_pose_set"] = {
            "position"   : {
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
            "friction"   : {
                "mu_roller_s_base": MU_ROLLER_S_BASE,
                "mu_pallet_s"     : PALLET_MU_S,
                "mu_eff_base"     : MU_ROLLER_BASE,
                "mu_eff_range"    : [
                    round(MU_ROLLER_BASE*(1-MU_RAND_RANGE), 4),
                    round(MU_ROLLER_BASE*(1+MU_RAND_RANGE), 4),
                ],
            },
            "critical"   : {
                "Y_critical_mm"   : round(Y_CRITICAL*1000, 1),
                "Yaw_critical_deg": round(YAW_CRITICAL, 1),
            },
            "stats"      : {
                "n_safe"    : len(safe),
                "total_grid": len(Y_GRID) * len(YAW_GRID),
                "safe_ratio": round(len(safe)/(len(Y_GRID)*len(YAW_GRID)), 3),
            },
        }
        save_log()
        print(f"  Y 安全范围   : {min(y_vals):.1f} ~ {max(y_vals):.1f} mm")
        print(f"  Yaw 安全范围 : {min(yaw_vals):.1f} ~ {max(yaw_vals):.1f} °")
        print(f"  安全格数     : {len(safe)}/{len(Y_GRID)*len(YAW_GRID)}"
              f"  ({len(safe)/(len(Y_GRID)*len(YAW_GRID))*100:.0f}%)")
    else:
        print("  ⚠ 网格中未找到成功率≥80% 的格子")
        print("  → 所有 trial 数据已保存，可在 all_trials 字段中查看原始结果")
        save_log()

    # 同时保存一份"全结果不过滤"的视图，方便手动筛选
    print(f"\n  ✓ 全部结果（含失败）已保存到:\n    {SAVE_PATH}")
    print(f"  字段说明:")
    print(f"    all_trials  → 每一次 trial 的原始记录（result/final_x/final_v）")
    print(f"    step2/3/4   → 按搜索步骤汇总的成功率")
    print(f"    safe_poses  → 成功率≥80% 的格子列表")
    print(f"\n✅ 搜索完成")


asyncio.ensure_future(main())
