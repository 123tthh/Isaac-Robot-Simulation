"""
step3_search_pitch.py  ── Isaac Sim 5.1 · Script Editor (Standalone)
═══════════════════════════════════════════════════════════════════
Step 3: 读 Step1+2 的 Y / Yaw 安全区间，略微抬高 Z（3mm）
        ★ 只对 Pitch 进行扫描（μ、Y、Yaw 都不随机化）

策略：
  • Y 固定为 Step1 区间中点
  • Yaw 固定为 Step2 区间中点
  • 只 Pitch 在变（从 4° 到 25°，步长 3°）
  • 这样 Pitch 的安全区间是"在最代表性的 Y/Yaw 配置下"的结果
  • 每个 Pitch 跑 N_TRIALS 次，仅用于确认可重复性

如果想额外验证 Y×Yaw 组合的 robustness，可在 Step3 之后另写一个
专门的 cross-validation 脚本，不应混在 Pitch 搜索里。

输出：~/Desktop/user_scripts/result/safe_pose_pitch.json
═══════════════════════════════════════════════════════════════════
"""

import os
import sys
import json
import math
import asyncio
import numpy as np

import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, UsdPhysics, Gf

# ═══════════════════════════════════════════════════════════════
# 全局常量 & 仿真参数
# ═══════════════════════════════════════════════════════════════

# 已经标定好的摩擦
MU_ROLLER_S_BASE = 0.060
PALLET_MU_S      = 0.400
MU_ROLLER_BASE   = MU_ROLLER_S_BASE * PALLET_MU_S
MU_RAND_RANGE    = 0.30     # 摩擦 DR ±30%（仅 Step4 用）

# 场景位置
PAL_X0_ACTUAL = 0.045
PAL_Z0_ACTUAL = 1.26751
PAL_Y0_CENTER = 0.00253
PITCH_BASE    = 4.0
PALLET_PATH   = "/World/Pallet/CollisionProxy"

# 仿真参数
PHYSICS_DT        = 1.0 / 60.0
SIM_DURATION      = 8.0
STUCK_SPEED       = 0.005    # m/s
STUCK_DURATION    = 3.0      # 静止持续超过这么久 → 判 stuck 提前终止
POSE_SETTLE_STEPS = 5

# 成功判定
CONVEYOR_END_X = 0.500
SUCCESS_X      = CONVEYOR_END_X * 0.9   # 0.45m

# 输出目录
RESULT_DIR = os.path.expanduser("~/Desktop/user_scripts/result")


# ═══════════════════════════════════════════════════════════════
# 场景句柄容器
# ═══════════════════════════════════════════════════════════════

class SceneHandle:
    """
    整个脚本运行期间只建一次，避免 SingleRigidPrim / World 反复创建
    导致的物理句柄失效问题。
    """
    def __init__(self):
        self.stage        = None
        self.proxy        = None
        self.pallet_prim  = None
        self.pallet_rb    = None
        self.world        = None
        self.timeline     = None
        self.app          = None

    async def setup(self):
        """建立 World + SingleRigidPrim，只做一次。"""
        self.stage = omni.usd.get_context().get_stage()
        self.proxy = self.stage.GetPrimAtPath(PALLET_PATH)
        if not self.proxy.IsValid():
            raise RuntimeError(f"找不到 {PALLET_PATH}")
        self.pallet_rb = UsdPhysics.RigidBodyAPI.Get(
            self.stage, self.proxy.GetPath())
        if not self.pallet_rb:
            raise RuntimeError(f"{PALLET_PATH} 无 RigidBodyAPI")

        self.timeline = omni.timeline.get_timeline_interface()
        self.app      = omni.kit.app.get_app()

        # World 初始化（全局单例）
        from isaacsim.core.api.world import World
        if World.instance() is not None:
            World.instance().clear_instance()
        self.world = World(
            physics_dt=PHYSICS_DT,
            rendering_dt=PHYSICS_DT,
            stage_units_in_meters=1.0,
        )
        await self.world.initialize_simulation_context_async()

        # 预先跑一次空 reset，让 PhysX 热身（之后每个 trial 才稳定）
        await self.world.reset_async()

        # SingleRigidPrim 只绑定一次，句柄全程复用
        try:
            from isaacsim.core.prims import SingleRigidPrim
        except ImportError:
            from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
        self.pallet_prim = SingleRigidPrim(
            prim_path=PALLET_PATH, name="pallet_shared")

        # 停掉 timeline，准备第一次 trial
        self.timeline.stop()
        print(f"  ✓ SceneHandle 初始化完成")


# ═══════════════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════════════

def set_pallet_pose(sh, x, y, z, roll_deg, pitch_deg, yaw_deg):
    """
    ★ 必须在 timeline.stop() 之后、reset_async() 之后调用。
    写 USD 位姿 + 清零速度。PhysX 在下一次推帧时读取。
    """
    xf = UsdGeom.XformCommonAPI(sh.proxy)
    xf.SetTranslate(Gf.Vec3d(float(x), float(y), float(z)))
    xf.SetRotate(
        Gf.Vec3f(float(roll_deg), float(pitch_deg), float(yaw_deg)),
        UsdGeom.XformCommonAPI.RotationOrderXYZ,
    )
    if sh.pallet_rb:
        sh.pallet_rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        sh.pallet_rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))


def set_roller_mu(sh, mu_s):
    """设置滚轮静摩擦 μs，动摩擦取 0.7×μs。"""
    mat = sh.stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return False
    m = UsdPhysics.MaterialAPI.Get(sh.stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(float(mu_s))
        m.GetDynamicFrictionAttr().Set(float(mu_s) * 0.7)
    return True


def extract_safe_range(values, rates, threshold=0.8):
    """
    输入：一维参数序列 values 和对应成功率 rates。
    返回：满足 rate≥threshold 的连续区间 [v_min, v_max]。
    若没有连续区间，返回 None。
    """
    safe_idx = [i for i, r in enumerate(rates) if r >= threshold]
    if not safe_idx:
        return None
    # 取最长连续段
    longest_start = longest_end = safe_idx[0]
    cur_start = safe_idx[0]
    prev = safe_idx[0]
    for i in safe_idx[1:]:
        if i == prev + 1:
            if i - cur_start > longest_end - longest_start:
                longest_end = i
                longest_start = cur_start
        else:
            cur_start = i
        prev = i
    # 再检查最后一段
    if prev - cur_start > longest_end - longest_start:
        longest_start = cur_start
        longest_end = prev
    return [float(values[longest_start]), float(values[longest_end])]


async def run_one_trial(sh, *, y, yaw_deg, pitch_deg=None,
                        z_offset=0.0, mu_roller_s=None, tag=""):
    """
    执行一次下滑试验。
    """
    if pitch_deg is None:
        pitch_deg = PITCH_BASE
    if mu_roller_s is None:
        mu_roller_s = MU_ROLLER_S_BASE

    # 1. 停 timeline
    if sh.timeline.is_playing():
        sh.timeline.stop()

    # 2. 设摩擦（Stop 状态下写材质，下一次 play 生效）
    set_roller_mu(sh, mu_roller_s)

    # 3. ★ 核心：先 reset 恢复场景，再写位姿
    await sh.world.reset_async()

    # 4. reset 完成后写目标位姿
    set_pallet_pose(
        sh,
        x        = PAL_X0_ACTUAL,
        y        = PAL_Y0_CENTER + y,
        z        = PAL_Z0_ACTUAL + z_offset,
        roll_deg = 0.0,
        pitch_deg= pitch_deg,
        yaw_deg  = yaw_deg,
    )

    # 5. Settle 帧（让 PhysX 读新位姿 + 接触稳定）
    for _ in range(POSE_SETTLE_STEPS):
        await sh.app.next_update_async()

    # 6. 读起始位置（验证 reset 是否生效）
    pos0, _ = sh.pallet_prim.get_world_pose()
    start_x = float(pos0[0])

    # 7. 推帧 + 实时判定
    n_steps       = int(SIM_DURATION / PHYSICS_DT)
    stuck_frames  = int(STUCK_DURATION / PHYSICS_DT)
    stuck_counter = 0
    result        = "timeout"
    final_x       = start_x
    final_v       = 0.0
    duration_used = 0.0

    for step in range(n_steps):
        await sh.app.next_update_async()

        pos, _ = sh.pallet_prim.get_world_pose()
        vel    = sh.pallet_prim.get_linear_velocity()
        x      = float(pos[0])
        v_mag  = float((vel[0]**2 + vel[1]**2 + vel[2]**2) ** 0.5)
        final_x = x
        final_v = v_mag
        duration_used = (step + 1) * PHYSICS_DT

        # 成功：前进到终点
        if x >= SUCCESS_X:
            result = "success"
            break

        # 卡住：持续静止
        if v_mag < STUCK_SPEED:
            stuck_counter += 1
        else:
            stuck_counter = 0
        if stuck_counter >= stuck_frames:
            result = "stuck"
            break

    # 8. 停
    sh.timeline.stop()

    print(f"      [{tag}] start_x={start_x:.4f} "
          f"→ {result}  (x={final_x:.4f}, v={final_v:.5f}, t={duration_used:.1f}s)")

    return {
        "result"  : result,
        "final_x" : round(final_x, 5),
        "final_v" : round(final_v, 5),
        "duration": round(duration_used, 2),
        "start_x" : round(start_x, 5),
    }


# ═══════════════════════════════════════════════════════════════
# Step 3 搜索流程
# ═══════════════════════════════════════════════════════════════

PITCH_SEARCH = np.arange(4.0, 25.0 + 0.01, 3.0)   # 4, 7, 10, 13, 16, 19, 22, 25
Z_OFFSET     = 0.003                              # 抬高 3mm
N_TRIALS     = 5

Y_INPUT   = os.path.join(RESULT_DIR, "safe_pose_y.json")
YAW_INPUT = os.path.join(RESULT_DIR, "safe_pose_yaw.json")
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_pitch.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 3] Pitch 扫描")
    print("  ★ 只扫描 Pitch，Y/Yaw 固定为前两步区间中点，μ 固定")
    print("═" * 62)

    # ── 读取前两步的安全区间 ────────────────────────────────
    if not os.path.exists(Y_INPUT) or not os.path.exists(YAW_INPUT):
        print(f"✗ 缺少 step1/step2 的输出，请先依次运行")
        return
    with open(Y_INPUT) as f:
        y_range = json.load(f).get("safe_range_mm") or [0.0, 0.0]
    with open(YAW_INPUT) as f:
        yaw_range = json.load(f).get("safe_range_deg") or [0.0, 0.0]

    # Y, Yaw 都取各自区间的中点
    Y_FIXED_MM   = (y_range[0]   + y_range[1])   / 2.0
    Y_FIXED_M    = Y_FIXED_MM / 1000.0
    YAW_FIXED    = (yaw_range[0] + yaw_range[1]) / 2.0

    print(f"  μ_roller_s   : {MU_ROLLER_S_BASE}（固定）")
    print(f"  Y  区间      : [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"  Y  固定      : {Y_FIXED_MM:.1f}mm")
    print(f"  Yaw 区间     : [{yaw_range[0]:.1f}, {yaw_range[1]:.1f}]°")
    print(f"  Yaw 固定     : {YAW_FIXED:.1f}°")
    print(f"  Pitch 搜索点 : {[round(float(p),1) for p in PITCH_SEARCH]}°")
    print(f"  Z 抬高       : {Z_OFFSET*1000:.1f} mm")
    print(f"  每点重复     : {N_TRIALS} 次")
    print(f"  输出         : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"               : "step3_Pitch",
            "note"               : "只扫描 Pitch，其余固定",
            "MU_ROLLER_S_BASE"   : MU_ROLLER_S_BASE,
            "MU_ROLLER_BASE"     : MU_ROLLER_BASE,
            "Y_input_range_mm"   : y_range,
            "Y_fixed_mm"         : round(Y_FIXED_MM, 1),
            "Yaw_input_range_deg": yaw_range,
            "Yaw_fixed_deg"      : round(YAW_FIXED, 1),
            "PITCH_SEARCH_deg"   : [round(float(p),1) for p in PITCH_SEARCH],
            "Z_OFFSET_m"         : Z_OFFSET,
            "N_TRIALS"           : N_TRIALS,
        },
        "trials"        : [],
        "summary"       : [],
        "safe_range_deg": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for pitch in PITCH_SEARCH:
        print(f"\n  ── Pitch = {pitch:.1f}° ──")
        successes = 0
        for t in range(N_TRIALS):
            rec = await run_one_trial(
                sh,
                y           = Y_FIXED_M,
                yaw_deg     = float(YAW_FIXED),
                pitch_deg   = float(pitch),
                z_offset    = Z_OFFSET,
                mu_roller_s = MU_ROLLER_S_BASE,
                tag         = f"Pitch={pitch:.1f}° #{t+1}/{N_TRIALS}",
            )
            rec["Pitch_deg"]  = round(float(pitch), 1)
            rec["Y_mm"]       = round(Y_FIXED_MM, 1)
            rec["Yaw_deg"]    = round(float(YAW_FIXED), 1)
            rec["Z_offset_m"] = Z_OFFSET
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
            save()

        rate = successes / N_TRIALS
        rates.append(rate)
        status = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  → 成功率 = {rate:.0%}  {status}")

        log["summary"].append({
            "Pitch_deg": round(float(pitch), 1),
            "rate"     : round(rate, 3),
        })
        save()

    values_deg = [round(float(p), 1) for p in PITCH_SEARCH]
    safe_range = extract_safe_range(values_deg, rates, threshold=0.8)
    log["safe_range_deg"] = safe_range
    save()

    print("\n" + "═" * 62)
    if safe_range:
        print(f"  ✓ Pitch 安全区间: [{safe_range[0]}, {safe_range[1]}]°")
    else:
        print("  ⚠ 未找到 Pitch 安全区间")
    print(f"  ✓ 结果已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())