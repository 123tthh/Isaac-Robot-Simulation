"""
step3_search_pitch.py  ── Isaac Sim 5.1 · Script Editor（完全自包含）
═══════════════════════════════════════════════════════════════════
Step 3: 读 Step1+2 的安全区间，Y/Yaw 固定为各自中点
        扫描 Pitch: 4° ~ 25°，步进 1°

本版改动（v2）：
  ★ Z 偏移按 Pitch 动态补偿（核心修正）
    Pitch=4° 是 roller 基准倾角，托盘平贴 roller。
    Pitch>4° 时托盘相对 roller 多倾 (Pitch-4°)，后缘会穿入。
    补偿公式：
        z_offset(P) = Z_BASE + tan(P - 4°) × (PALLET_DIM_X / 2)

    实例：
        Pitch=4°  → z=1.24mm   （贴合）
        Pitch=10° → z=1.24 + 9.72 = 10.96mm
        Pitch=15° → z=1.24 + 17.54 = 18.78mm
        Pitch=25° → z=1.24 + 34.73 = 35.97mm

  ★ Yaw 锁死检测（与 step1/2 一致）：
    |Δyaw| > 3° 持续 ≥ 0.5s → rail_lock，当失败处理

输出: projects/physics_parameters/results/safe_pose_pitch.json
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
import json
import math
import asyncio
import numpy as np

try:
    from scipy.spatial.transform import Rotation
except ImportError:
    raise RuntimeError("缺少 scipy 库。请 pip install scipy")

import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, UsdPhysics, Gf, UsdShade

# ═══════════════════════════════════════════════════════════════
# 常量
# ═══════════════════════════════════════════════════════════════

MU_ROLLER_S_BASE = 0.060
PALLET_MU_S      = 0.400

MU_RAIL_S = 0.60
MU_RAIL_D = 0.50

PAL_X0_ACTUAL = 0.045
PAL_Z0_ACTUAL = 1.254
PAL_Y0_CENTER = 0.0
PITCH_BASE    = 4.0
PALLET_PATH   = "/World/Pallet/CollisionProxy"

PALLET_DIM_X = 0.185
PALLET_DIM_Y = 0.265
PALLET_DIM_Z = 0.040

RAIL_PATHS = [
    "/World/ConveyorGroup/Rail_Left_forward",
    "/World/ConveyorGroup/Rail_Right_forward",
    "/World/ConveyorGroup/Rail_Left_backward",
    "/World/ConveyorGroup/Rail_Right_backward",
]
RAIL_MAT_PATH = "/World/Mat/RailFriction"

PHYSICS_DT        = 1.0 / 60.0
SIM_DURATION      = 8.0
STUCK_SPEED       = 0.005
STUCK_DURATION    = 3.0
POSE_SETTLE_STEPS = 5

SUCCESS_DX = 0.30
Z_BASE_M   = 0.00124

YAW_LOCK_THRESHOLD = 3.0
YAW_LOCK_DURATION  = 1 
YAW_LOCK_FRAMES    = int(YAW_LOCK_DURATION / PHYSICS_DT)

RESULT_DIR = str(_PHYSICS_ROOT / "results")


def pitch_z_offset(pitch_deg):
    """★ Pitch 抬高补偿：Pitch>4° 时自动抬起避免后缘穿入 roller。"""
    delta = max(0.0, pitch_deg - PITCH_BASE)
    return Z_BASE_M + math.tan(math.radians(delta)) * (PALLET_DIM_X / 2.0)


# ═══════════════════════════════════════════════════════════════
# SceneHandle
# ═══════════════════════════════════════════════════════════════

class SceneHandle:
    def __init__(self):
        self.stage = self.proxy = None
        self.pallet_prim = self.pallet_rb = None
        self.world = self.timeline = self.app = None

    async def setup(self):
        self.stage = omni.usd.get_context().get_stage()
        self.proxy = self.stage.GetPrimAtPath(PALLET_PATH)
        if not self.proxy.IsValid():
            raise RuntimeError(f"找不到 {PALLET_PATH}")
        self.pallet_rb = UsdPhysics.RigidBodyAPI.Get(self.stage, self.proxy.GetPath())
        if not self.pallet_rb:
            raise RuntimeError(f"{PALLET_PATH} 无 RigidBodyAPI")
        self.timeline = omni.timeline.get_timeline_interface()
        self.app = omni.kit.app.get_app()

        from isaacsim.core.api.world import World
        if World.instance() is not None:
            World.instance().clear_instance()
        self.world = World(physics_dt=PHYSICS_DT, rendering_dt=PHYSICS_DT,
                           stage_units_in_meters=1.0)
        await self.world.initialize_simulation_context_async()
        await self.world.reset_async()

        try:
            from isaacsim.core.prims import SingleRigidPrim
        except ImportError:
            from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
        self.pallet_prim = SingleRigidPrim(prim_path=PALLET_PATH, name="pallet_shared")

        self._ensure_rail_friction()
        self.timeline.stop()
        print("  ✓ SceneHandle 初始化完成（含导轨高摩擦材质）")

    def _ensure_rail_friction(self):
        rail_mat = self.stage.GetPrimAtPath(RAIL_MAT_PATH)
        if not rail_mat.IsValid():
            UsdShade.Material.Define(self.stage, RAIL_MAT_PATH)
            rail_mat = self.stage.GetPrimAtPath(RAIL_MAT_PATH)
            UsdPhysics.MaterialAPI.Apply(rail_mat)
        material = UsdShade.Material(rail_mat)
        bound = 0
        for rp in RAIL_PATHS:
            p = self.stage.GetPrimAtPath(rp)
            if not p.IsValid():
                continue
            if not p.HasAPI(UsdShade.MaterialBindingAPI):
                UsdShade.MaterialBindingAPI.Apply(p)
            UsdShade.MaterialBindingAPI(p).Bind(material)
            bound += 1
        mp = UsdPhysics.MaterialAPI.Get(self.stage, rail_mat.GetPath())
        if mp:
            mp.GetStaticFrictionAttr().Set(float(MU_RAIL_S))
            mp.GetDynamicFrictionAttr().Set(float(MU_RAIL_D))
        print(f"    ✓ 已绑定导轨摩擦材质到 {bound}/4 根")


# ═══════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════

def set_pallet_pose(sh, x, y, z, roll_deg, pitch_deg, yaw_deg):
    pos = np.array([float(x), float(y), float(z)])
    rx = Gf.Rotation(Gf.Vec3d(1, 0, 0), float(roll_deg))
    ry = Gf.Rotation(Gf.Vec3d(0, 1, 0), float(pitch_deg))
    rz = Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg))
    q = (rx * ry * rz).GetQuat()
    quat_wxyz = np.array([q.GetReal(),
                          q.GetImaginary()[0],
                          q.GetImaginary()[1],
                          q.GetImaginary()[2]])
    sh.pallet_prim.set_world_pose(position=pos, orientation=quat_wxyz)
    sh.pallet_prim.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
    sh.pallet_prim.set_angular_velocity(np.array([0.0, 0.0, 0.0]))


def set_roller_mu(sh, mu_s):
    mat = sh.stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return False
    m = UsdPhysics.MaterialAPI.Get(sh.stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(float(mu_s))
        m.GetDynamicFrictionAttr().Set(float(mu_s) * 0.7)
    return True


def get_yaw_from_quat(quat_wxyz):
    r = Rotation.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
    return float(r.as_euler('xyz', degrees=True)[2])


def extract_safe_range(values, rates, threshold=0.8):
    safe_idx = [i for i, r in enumerate(rates) if r >= threshold]
    if not safe_idx:
        return None
    longest_start = longest_end = cur_start = prev = safe_idx[0]
    for i in safe_idx[1:]:
        if i == prev + 1:
            if i - cur_start > longest_end - longest_start:
                longest_end = i
                longest_start = cur_start
        else:
            cur_start = i
        prev = i
    if prev - cur_start > longest_end - longest_start:
        longest_start = cur_start
        longest_end = prev
    return [float(values[longest_start]), float(values[longest_end])]


# ═══════════════════════════════════════════════════════════════
# run_one_trial
# ═══════════════════════════════════════════════════════════════

async def run_one_trial(sh, *, y, yaw_deg, pitch_deg,
                        z_offset, mu_roller_s, tag=""):
    if sh.timeline.is_playing():
        sh.timeline.stop()

    set_roller_mu(sh, mu_roller_s)
    await sh.world.reset_async()

    set_pallet_pose(sh,
        x=PAL_X0_ACTUAL, y=PAL_Y0_CENTER + y, z=PAL_Z0_ACTUAL + z_offset,
        roll_deg=0.0, pitch_deg=pitch_deg, yaw_deg=yaw_deg)

    for _ in range(POSE_SETTLE_STEPS):
        await sh.app.next_update_async()

    pos0, rot0 = sh.pallet_prim.get_world_pose()
    start_x   = float(pos0[0])
    start_yaw = get_yaw_from_quat(rot0)

    n_steps       = int(SIM_DURATION / PHYSICS_DT)
    stuck_frames  = int(STUCK_DURATION / PHYSICS_DT)
    stuck_counter = 0

    yaw_lock_counter = 0
    yaw_lock_triggered = False
    max_abs_yaw_drift = 0.0
    yaw_at_trigger = 0.0

    result        = "timeout"
    max_x         = start_x
    final_x       = start_x
    final_v       = 0.0
    duration_used = 0.0

    for step in range(n_steps):
        await sh.app.next_update_async()
        pos, rot = sh.pallet_prim.get_world_pose()
        vel      = sh.pallet_prim.get_linear_velocity()
        x        = float(pos[0])
        v_mag    = float((vel[0]**2 + vel[1]**2 + vel[2]**2) ** 0.5)
        cur_yaw  = get_yaw_from_quat(rot)
        yaw_drift = cur_yaw - start_yaw

        max_x = max(max_x, x)
        final_x = x
        final_v = v_mag
        duration_used = (step + 1) * PHYSICS_DT
        max_abs_yaw_drift = max(max_abs_yaw_drift, abs(yaw_drift))

        if abs(yaw_drift) > YAW_LOCK_THRESHOLD:
            yaw_lock_counter += 1
            if yaw_lock_counter >= YAW_LOCK_FRAMES and not yaw_lock_triggered:
                yaw_lock_triggered = True
                yaw_at_trigger = yaw_drift
                result = "rail_lock"
                break
        else:
            yaw_lock_counter = 0

        if max_x - start_x >= SUCCESS_DX:
            result = "success"; break

        if v_mag < STUCK_SPEED:
            stuck_counter += 1
        else:
            stuck_counter = 0
        if stuck_counter >= stuck_frames:
            result = "success" if (max_x - start_x >= SUCCESS_DX) else "stuck"
            break

    if result == "timeout" and max_x - start_x >= SUCCESS_DX:
        result = "success"

    sh.timeline.stop()

    dx_mm = (max_x - start_x) * 1000
    lock_note = ""
    if result == "rail_lock":
        lock_note = f" ⚠锁死 Δyaw={yaw_at_trigger:+.1f}°"
    elif max_abs_yaw_drift > YAW_LOCK_THRESHOLD / 2:
        lock_note = f" (maxΔyaw={max_abs_yaw_drift:.1f}°)"

    print(f"      [{tag}] dx={dx_mm:.1f}mm → {result}  "
          f"(v={final_v:.5f}, t={duration_used:.1f}s){lock_note}")

    return {
        "result"               : result,
        "start_yaw_deg"        : round(start_yaw, 2),
        "max_abs_yaw_drift_deg": round(max_abs_yaw_drift, 2),
        "yaw_at_trigger_deg"   : round(yaw_at_trigger, 2) if yaw_lock_triggered else None,
        "rail_lock_triggered"  : yaw_lock_triggered,
        "final_x"              : round(final_x, 5),
        "max_x"                : round(max_x, 5),
        "dx_mm"                : round(dx_mm, 2),
        "final_v"              : round(final_v, 5),
        "duration"             : round(duration_used, 2),
        "start_x"              : round(start_x, 5),
    }


# ═══════════════════════════════════════════════════════════════
# Step 3 参数 & 流程
# ═══════════════════════════════════════════════════════════════

PITCH_SEARCH = np.arange(4.0, 25.0 + 0.01, 1.0)
N_TRIALS     = 3

Y_INPUT   = os.path.join(RESULT_DIR, "safe_pose_y.json")
YAW_INPUT = os.path.join(RESULT_DIR, "safe_pose_yaw.json")
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_pitch.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 3] Pitch 扫描 (v2)")
    print(f"  Z 基础偏移: {Z_BASE_M*1000:.2f} mm （随 Pitch 动态抬高）")
    print(f"  锁死阈值  : |Δyaw| > {YAW_LOCK_THRESHOLD}° 持续 > {YAW_LOCK_DURATION}s")
    print("═" * 62)

    if not os.path.exists(Y_INPUT) or not os.path.exists(YAW_INPUT):
        print("✗ 缺少 step1/step2 输出"); return
    with open(Y_INPUT) as f:
        y_range = json.load(f).get("safe_range_mm") or [0.0, 0.0]
    with open(YAW_INPUT) as f:
        yaw_range = json.load(f).get("safe_range_deg") or [0.0, 0.0]

    Y_FIXED_MM = (y_range[0] + y_range[1]) / 2.0
    Y_FIXED_M  = Y_FIXED_MM / 1000.0
    YAW_FIXED  = (yaw_range[0] + yaw_range[1]) / 2.0

    print(f"  Y 固定  : {Y_FIXED_MM:+.1f}mm")
    print(f"  Yaw 固定: {YAW_FIXED:+.1f}°")
    print(f"  Pitch 范围: {PITCH_SEARCH[0]:.0f} ~ {PITCH_SEARCH[-1]:.0f}°"
          f"  步进1° 共 {len(PITCH_SEARCH)} 点")
    print(f"  每点重复: {N_TRIALS} 次")

    # 打印 Pitch→Z 对应表（前几个点）
    print(f"\n  Pitch→Z 抬高对照表（前 6 点）：")
    for p in PITCH_SEARCH[:6]:
        zo = pitch_z_offset(float(p))
        print(f"    Pitch={p:5.1f}°  →  z_offset={zo*1000:6.2f} mm")
    print(f"    Pitch={PITCH_SEARCH[-1]:5.1f}°  →  z_offset="
          f"{pitch_z_offset(float(PITCH_SEARCH[-1]))*1000:6.2f} mm")
    print(f"  输出    : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step": "step3_Pitch_v2",
            "SUCCESS_DX_m": SUCCESS_DX, "Z_BASE_m": Z_BASE_M,
            "Z_dynamic_formula": "Z_BASE + tan(P-4°) * PALLET_DIM_X/2",
            "MU_ROLLER_S_BASE": MU_ROLLER_S_BASE,
            "MU_RAIL_S": MU_RAIL_S, "MU_RAIL_D": MU_RAIL_D,
            "YAW_LOCK_THRESHOLD": YAW_LOCK_THRESHOLD,
            "YAW_LOCK_DURATION_s": YAW_LOCK_DURATION,
            "Y_fixed_mm": round(Y_FIXED_MM, 1),
            "Yaw_fixed_deg": round(YAW_FIXED, 1),
            "Pitch_range_deg": [round(float(PITCH_SEARCH[0]),1),
                                round(float(PITCH_SEARCH[-1]),1)],
            "Pitch_step_deg": 1.0,
            "N_TRIALS": N_TRIALS,
        },
        "trials": [], "summary": [], "safe_range_deg": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for pitch in PITCH_SEARCH:
        p_r = round(float(pitch), 1)
        z_off = pitch_z_offset(float(pitch))  # ★ 动态 Z 抬高
        successes = locks = stucks = 0
        for t in range(N_TRIALS):
            rec = await run_one_trial(
                sh, y=Y_FIXED_M,
                yaw_deg=float(YAW_FIXED), pitch_deg=float(pitch),
                z_offset=z_off,
                mu_roller_s=MU_ROLLER_S_BASE,
                tag=f"P={p_r:.1f}° z={z_off*1000:.1f}mm #{t+1}")
            rec["Pitch_deg"] = p_r
            rec["Y_mm"] = round(Y_FIXED_MM, 1)
            rec["Yaw_deg"] = round(float(YAW_FIXED), 1)
            rec["Z_offset_m"] = round(z_off, 5)
            log["trials"].append(rec)
            if   rec["result"] == "success":   successes += 1
            elif rec["result"] == "rail_lock": locks += 1
            elif rec["result"] == "stuck":     stucks += 1

        rate = successes / N_TRIALS
        rates.append(rate)
        if rate >= 0.8: s = "✓"
        elif locks >= N_TRIALS / 2: s = "🔒lock"
        elif stucks >= N_TRIALS / 2: s = "✗stuck"
        else: s = "△mix"
        print(f"  Pitch={p_r:5.1f}° z={z_off*1000:5.2f}mm → {rate:.0%}  "
              f"(ok={successes} lock={locks} stuck={stucks}) {s}")
        log["summary"].append({
            "Pitch_deg": p_r, "Z_offset_mm": round(z_off*1000, 2),
            "rate": round(rate, 3),
            "n_success": successes, "n_rail_lock": locks, "n_stuck": stucks,
        })
        save()

    values = [round(float(p), 1) for p in PITCH_SEARCH]
    log["safe_range_deg"] = extract_safe_range(values, rates, 0.8)
    save()

    print("\n" + "═" * 62)
    if log["safe_range_deg"]:
        print(f"  ✓ Pitch 安全区间: {log['safe_range_deg']}°")
    else:
        print("  ⚠ 未找到 Pitch 安全区间")
    print(f"  ✓ 已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
