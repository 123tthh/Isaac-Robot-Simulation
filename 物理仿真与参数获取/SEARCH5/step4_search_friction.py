"""
step4_search_friction.py  ── Isaac Sim 5.1 · Script Editor（完全自包含）
═══════════════════════════════════════════════════════════════════
Step 4: 读 Step1/2/3 的安全区间，Y/Yaw/Pitch 固定为各自中点
        只扫描 μ_roller_s: 0.020 ~ 0.100，步进 0.005
  • 输出: ~/Desktop/user_scripts/result/safe_pose.json（最终产物）
═══════════════════════════════════════════════════════════════════
"""

import os
import json
import asyncio
import numpy as np

import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, UsdPhysics, Gf

# ═══════════════════════════════════════════════════════════════
# 共享常量
# ═══════════════════════════════════════════════════════════════

MU_ROLLER_S_BASE = 0.060
PALLET_MU_S      = 0.400
MU_ROLLER_BASE   = MU_ROLLER_S_BASE * PALLET_MU_S

PAL_X0_ACTUAL = 0.045
PAL_Z0_ACTUAL = 1.26751
PAL_Y0_CENTER = 0.00253
PITCH_BASE    = 4.0
PALLET_PATH   = "/World/Pallet/CollisionProxy"

PHYSICS_DT        = 1.0 / 60.0
SIM_DURATION      = 8.0
STUCK_SPEED       = 0.005
STUCK_DURATION    = 3.0
POSE_SETTLE_STEPS = 5

# ★ 成功判定：位移 ≥ 0.30m（覆盖被挡板/限位器顶停的情况）
SUCCESS_DX = 0.30

RESULT_DIR = os.path.expanduser("~/Desktop/user_scripts/result")


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
        self.timeline.stop()
        print("  ✓ SceneHandle 初始化完成")


def set_pallet_pose(sh, x, y, z, roll_deg, pitch_deg, yaw_deg):
    xf = UsdGeom.XformCommonAPI(sh.proxy)
    xf.SetTranslate(Gf.Vec3d(float(x), float(y), float(z)))
    xf.SetRotate(Gf.Vec3f(float(roll_deg), float(pitch_deg), float(yaw_deg)),
                 UsdGeom.XformCommonAPI.RotationOrderXYZ)
    if sh.pallet_rb:
        sh.pallet_rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        sh.pallet_rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))


def set_roller_mu(sh, mu_s):
    mat = sh.stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return False
    m = UsdPhysics.MaterialAPI.Get(sh.stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(float(mu_s))
        m.GetDynamicFrictionAttr().Set(float(mu_s) * 0.7)
    return True


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
# run_one_trial  ★ 位移优先判定
#
# 判定逻辑（按优先级）：
#   1. dx ≥ SUCCESS_DX (0.30m) → success（立即终止）
#   2. 速度 < STUCK_SPEED 持续 ≥ STUCK_DURATION 秒
#      → 再看 dx：若 dx ≥ SUCCESS_DX → success（被挡板停住）
#                否则 → stuck（真的卡住）
#   3. 跑满 SIM_DURATION → timeout
# ═══════════════════════════════════════════════════════════════

async def run_one_trial(sh, *, y, yaw_deg, pitch_deg=None,
                        z_offset=0.0, mu_roller_s=None, tag=""):
    if pitch_deg is None:
        pitch_deg = PITCH_BASE
    if mu_roller_s is None:
        mu_roller_s = MU_ROLLER_S_BASE

    if sh.timeline.is_playing():
        sh.timeline.stop()

    set_roller_mu(sh, mu_roller_s)
    await sh.world.reset_async()

    set_pallet_pose(sh,
        x=PAL_X0_ACTUAL, y=PAL_Y0_CENTER + y, z=PAL_Z0_ACTUAL + z_offset,
        roll_deg=0.0, pitch_deg=pitch_deg, yaw_deg=yaw_deg)

    for _ in range(POSE_SETTLE_STEPS):
        await sh.app.next_update_async()

    pos0, _ = sh.pallet_prim.get_world_pose()
    start_x = float(pos0[0])

    n_steps       = int(SIM_DURATION / PHYSICS_DT)
    stuck_frames  = int(STUCK_DURATION / PHYSICS_DT)
    stuck_counter = 0
    result        = "timeout"
    max_x         = start_x
    final_x       = start_x
    final_v       = 0.0
    duration_used = 0.0

    for step in range(n_steps):
        await sh.app.next_update_async()

        pos, _ = sh.pallet_prim.get_world_pose()
        vel    = sh.pallet_prim.get_linear_velocity()
        x      = float(pos[0])
        v_mag  = float((vel[0]**2 + vel[1]**2 + vel[2]**2) ** 0.5)
        max_x         = max(max_x, x)
        final_x       = x
        final_v       = v_mag
        duration_used = (step + 1) * PHYSICS_DT

        # ★ 优先级1：位移够了 → 立即成功
        if max_x - start_x >= SUCCESS_DX:
            result = "success"
            break

        # 卡住计数
        if v_mag < STUCK_SPEED:
            stuck_counter += 1
        else:
            stuck_counter = 0

        # ★ 优先级2：持续静止 → 看位移决定成功/卡住
        if stuck_counter >= stuck_frames:
            if max_x - start_x >= SUCCESS_DX:
                result = "success"
            else:
                result = "stuck"
            break

    # 跑满也检查一次位移
    if result == "timeout" and max_x - start_x >= SUCCESS_DX:
        result = "success"

    sh.timeline.stop()

    dx_mm = (max_x - start_x) * 1000
    print(f"      [{tag}] dx={dx_mm:.1f}mm → {result}"
          f"  (v={final_v:.5f}, t={duration_used:.1f}s)")

    return {
        "result"  : result,
        "final_x" : round(final_x, 5),
        "max_x"   : round(max_x, 5),
        "dx_mm"   : round(dx_mm, 2),
        "final_v" : round(final_v, 5),
        "duration": round(duration_used, 2),
        "start_x" : round(start_x, 5),
    }

# ═══════════════════════════════════════════════════════════════
# Step 4 参数 & 流程
# ═══════════════════════════════════════════════════════════════

MU_SEARCH = np.arange(0.020, 0.100 + 0.001, 0.005)  # 0.020~0.100 步进0.005
Z_OFFSET  = 0.003
N_TRIALS  = 5

Y_INPUT     = os.path.join(RESULT_DIR, "safe_pose_y.json")
YAW_INPUT   = os.path.join(RESULT_DIR, "safe_pose_yaw.json")
PITCH_INPUT = os.path.join(RESULT_DIR, "safe_pose_pitch.json")
SAVE_PATH   = os.path.join(RESULT_DIR, "safe_pose.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 4] μ_roller_s 扫描 → 最终 Safe Pose Set")
    print("  ★ 只扫描 μ，Y/Yaw/Pitch 固定为各自区间中点")
    print("═" * 62)

    for p in [Y_INPUT, YAW_INPUT, PITCH_INPUT]:
        if not os.path.exists(p):
            print(f"✗ 缺少 {p}"); return

    with open(Y_INPUT) as f:
        y_range = json.load(f).get("safe_range_mm") or [0.0, 0.0]
    with open(YAW_INPUT) as f:
        yaw_range = json.load(f).get("safe_range_deg") or [0.0, 0.0]
    with open(PITCH_INPUT) as f:
        pitch_range = json.load(f).get("safe_range_deg") or [4.0, 4.0]

    Y_FIXED_MM  = (y_range[0] + y_range[1]) / 2.0
    Y_FIXED_M   = Y_FIXED_MM / 1000.0
    YAW_FIXED   = (yaw_range[0] + yaw_range[1]) / 2.0
    PITCH_FIXED = (pitch_range[0] + pitch_range[1]) / 2.0

    print(f"  Y={Y_FIXED_MM:.1f}mm  Yaw={YAW_FIXED:.1f}°  Pitch={PITCH_FIXED:.1f}°")
    print(f"  μ 范围: {MU_SEARCH[0]:.3f} ~ {MU_SEARCH[-1]:.3f}  步进0.005"
          f"  共 {len(MU_SEARCH)} 点")
    print(f"  每点重复: {N_TRIALS} 次")
    print(f"  输出: {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step": "step4_Friction",
            "Y_fixed_mm": round(Y_FIXED_MM,1),
            "Yaw_fixed_deg": round(YAW_FIXED,1),
            "Pitch_fixed_deg": round(PITCH_FIXED,1),
            "MU_range": [round(float(MU_SEARCH[0]),4), round(float(MU_SEARCH[-1]),4)],
            "Z_OFFSET_m": Z_OFFSET, "N_TRIALS": N_TRIALS,
        },
        "trials": [], "summary": [],
        "safe_mu_s_range": None,
        "final_safe_pose": {},
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for mu_s in MU_SEARCH:
        mu_r = round(float(mu_s), 4)
        successes = 0
        for t in range(N_TRIALS):
            rec = await run_one_trial(sh, y=Y_FIXED_M,
                yaw_deg=float(YAW_FIXED), pitch_deg=float(PITCH_FIXED),
                z_offset=Z_OFFSET, mu_roller_s=float(mu_s),
                tag=f"μ={mu_r} #{t+1}")
            rec["mu_roller_s"] = mu_r
            rec["mu_eff"] = round(mu_r * PALLET_MU_S, 4)
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
        rate = successes / N_TRIALS
        rates.append(rate)
        s = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  μ={mu_r:.4f} (eff={mu_r*PALLET_MU_S:.4f}) → {rate:.0%} {s}")
        log["summary"].append({"mu_roller_s": mu_r, "rate": round(rate,3)})
        save()

    mu_vals = [round(float(m),4) for m in MU_SEARCH]
    safe_mu = extract_safe_range(mu_vals, rates, 0.8)
    log["safe_mu_s_range"] = safe_mu

    log["final_safe_pose"] = {
        "position": {"X_m": PAL_X0_ACTUAL, "Y_range_mm": y_range,
                     "Z_m": PAL_Z0_ACTUAL + Z_OFFSET},
        "orientation": {"Roll_deg": 0.0, "Pitch_range_deg": pitch_range,
                        "Yaw_range_deg": yaw_range},
        "friction": {"mu_roller_s_range": safe_mu,
                     "mu_pallet_s": PALLET_MU_S},
    }
    save()

    print("\n" + "═" * 62)
    print("  ★ 最终 Safe Pose Set")
    print(f"    Y     : {y_range} mm")
    print(f"    Yaw   : {yaw_range}°")
    print(f"    Pitch : {pitch_range}°")
    print(f"    μ_s   : {safe_mu or '(未收敛)'}")
    print(f"\n  ✓ 写入: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
