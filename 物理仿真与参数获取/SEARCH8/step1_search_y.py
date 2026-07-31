"""
step1_search_y.py  ── Isaac Sim 5.1 · Script Editor（完全自包含）
═══════════════════════════════════════════════════════════════════
Step 1: 固定 μ=0.060、Yaw=0°、Pitch=4°，只扫描 Y 偏心

本版核心改动（v2）：
  ★ Z_OFFSET = 1.24mm（托盘贴滚轮释放，模拟真实夹爪 release）
    抬高过多会引入不真实的下落冲量，导致 sim2real 偏差。

  ★ 新增：Yaw 锁死力矩检测
    data_plan.md 指出真实物理中托盘接触导轨侧面会瞬间锁死。
    PhysX 仿真即使加了 μ_rail=0.6，低 dt + 细长接触时侧向力不一定
    稳定解算，托盘可能"擦过"——这是假成功。

    检测逻辑：
      - 全程每帧记录 yaw_trajectory
      - 统计 |Δyaw| > YAW_LOCK_THRESHOLD（3°）持续多少帧
      - 若持续帧数 ≥ YAW_LOCK_FRAMES（0.5s = 25帧）→ 判 rail_lock
      - rail_lock 视为 sim2real 不可靠的"假成功"，当作失败处理

判定优先级：
  1. |yaw_drift| 持续 > 阈值 → rail_lock（真实会卡死的假成功）
  2. dx ≥ SUCCESS_DX (0.30m) → success
  3. |V| < STUCK_SPEED 持续 > STUCK_DURATION → stuck
  4. 跑满 SIM_DURATION → timeout

输出: ~/Desktop/user_scripts/result/safe_pose_y.json
═══════════════════════════════════════════════════════════════════
"""

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
MU_ROLLER_BASE   = MU_ROLLER_S_BASE * PALLET_MU_S

# 导轨侧面摩擦（模拟真实塑料-塑料）
MU_RAIL_S = 0.60
MU_RAIL_D = 0.50

PAL_X0_ACTUAL = 0.045
PAL_Z0_ACTUAL = 1.254
PAL_Y0_CENTER = 0.0
PITCH_BASE    = 4.0
PALLET_PATH   = "/World/Pallet/CollisionProxy"

# 托盘尺寸（用于后续 step 的 pitch 抬高补偿）
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

# 成功判定
SUCCESS_DX = 0.30

# ★ Step1 专属：贴滚轮释放，模拟真实夹爪
Z_OFFSET_M = 0.00124

# ═════ Yaw 锁死检测参数 ═════
YAW_LOCK_THRESHOLD = 3.0      # |Δyaw| 超过此角度视为异常接触导轨（°）
YAW_LOCK_DURATION  = 0.5      # 持续超过此秒数 → 判定锁死（s）
YAW_LOCK_FRAMES    = int(YAW_LOCK_DURATION / PHYSICS_DT)  # = 18 帧

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

        # 一次性建立导轨侧面高摩擦材质
        self._ensure_rail_friction()

        self.timeline.stop()
        print("  ✓ SceneHandle 初始化完成（含导轨高摩擦材质）")

    def _ensure_rail_friction(self):
        """在场景中建立导轨 PhysicsMaterial，绑定给 4 根导轨。只做一次。"""
        rail_mat = self.stage.GetPrimAtPath(RAIL_MAT_PATH)
        if not rail_mat.IsValid():
            UsdShade.Material.Define(self.stage, RAIL_MAT_PATH)
            rail_mat = self.stage.GetPrimAtPath(RAIL_MAT_PATH)
            UsdPhysics.MaterialAPI.Apply(rail_mat)

        material = UsdShade.Material(rail_mat)
        bound_count = 0
        for rp in RAIL_PATHS:
            p = self.stage.GetPrimAtPath(rp)
            if not p.IsValid():
                print(f"    ⚠ 导轨 prim 不存在: {rp}")
                continue
            if not p.HasAPI(UsdShade.MaterialBindingAPI):
                UsdShade.MaterialBindingAPI.Apply(p)
            UsdShade.MaterialBindingAPI(p).Bind(material)
            bound_count += 1

        mp = UsdPhysics.MaterialAPI.Get(self.stage, rail_mat.GetPath())
        if mp:
            mp.GetStaticFrictionAttr().Set(float(MU_RAIL_S))
            mp.GetDynamicFrictionAttr().Set(float(MU_RAIL_D))
        print(f"    ✓ 已绑定导轨摩擦材质到 {bound_count}/4 根"
              f"  μs={MU_RAIL_S} μd={MU_RAIL_D}")


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
    """从 [w,x,y,z] 四元数提取 Yaw 角（°）。"""
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
# run_one_trial  ★ 含 Yaw 锁死检测
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

    # Settle 后的基线状态
    pos0, rot0 = sh.pallet_prim.get_world_pose()
    start_x   = float(pos0[0])
    start_yaw = get_yaw_from_quat(rot0)

    # 主仿真循环
    n_steps       = int(SIM_DURATION / PHYSICS_DT)
    stuck_frames  = int(STUCK_DURATION / PHYSICS_DT)
    stuck_counter = 0

    # ★ Yaw 锁死检测状态
    yaw_lock_counter = 0        # 连续超阈值帧数
    yaw_lock_triggered = False  # 曾触发锁死
    max_abs_yaw_drift = 0.0     # 整个过程的最大偏转量
    yaw_at_trigger = 0.0        # 触发时的 yaw 值

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

        max_x         = max(max_x, x)
        final_x       = x
        final_v       = v_mag
        duration_used = (step + 1) * PHYSICS_DT
        max_abs_yaw_drift = max(max_abs_yaw_drift, abs(yaw_drift))

        # ★ 优先级 1：Yaw 锁死检测（真实物理的死锁力矩）
        if abs(yaw_drift) > YAW_LOCK_THRESHOLD:
            yaw_lock_counter += 1
            if yaw_lock_counter >= YAW_LOCK_FRAMES and not yaw_lock_triggered:
                yaw_lock_triggered = True
                yaw_at_trigger = yaw_drift
                result = "rail_lock"
                break   # 立即终止：sim 即使能继续，real 也会卡死
        else:
            yaw_lock_counter = 0

        # 优先级 2：位移到位
        if max_x - start_x >= SUCCESS_DX:
            result = "success"
            break

        # 优先级 3：速度持续为零
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
        lock_note = f" ⚠锁死于Yaw={yaw_at_trigger:+.1f}°"
    elif max_abs_yaw_drift > YAW_LOCK_THRESHOLD / 2:
        lock_note = f" (最大Yaw偏={max_abs_yaw_drift:.1f}°)"

    print(f"      [{tag}] dx={dx_mm:.1f}mm → {result}  "
          f"(v={final_v:.5f}, t={duration_used:.1f}s){lock_note}")

    return {
        "result"             : result,
        "start_yaw_deg"      : round(start_yaw, 2),
        "max_abs_yaw_drift_deg": round(max_abs_yaw_drift, 2),
        "yaw_at_trigger_deg" : round(yaw_at_trigger, 2) if yaw_lock_triggered else None,
        "rail_lock_triggered": yaw_lock_triggered,
        "final_x"            : round(final_x, 5),
        "max_x"              : round(max_x, 5),
        "dx_mm"              : round(dx_mm, 2),
        "final_v"            : round(final_v, 5),
        "duration"           : round(duration_used, 2),
        "start_x"            : round(start_x, 5),
    }


# ═══════════════════════════════════════════════════════════════
# Step 1 参数 & 流程
# ═══════════════════════════════════════════════════════════════

Y_SEARCH  = np.arange(-0.025, 0.025 + 0.00001, 0.001)   # -25~+25mm 步进1mm
N_TRIALS  = 5              # 每中情况重复次数
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_y.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 1] Y 偏心扫描 (v2)")
    print(f"  Z 偏移  : {Z_OFFSET_M*1000:.2f} mm (贴滚轮释放)")
    print(f"  判定    : 位移 ≥ {SUCCESS_DX*1000:.0f}mm → success")
    print(f"  锁死阈值: |Δyaw| > {YAW_LOCK_THRESHOLD}° 持续 > {YAW_LOCK_DURATION}s"
          f"  → rail_lock（假成功，sim2real 不可靠）")
    print(f"  Roller μ: {MU_ROLLER_S_BASE}")
    print(f"  Rail μ  : {MU_RAIL_S} / {MU_RAIL_D}")
    print("═" * 62)
    print(f"  Y 范围  : {Y_SEARCH[0]*1000:+.0f} ~ {Y_SEARCH[-1]*1000:+.0f} mm"
          f"  步进 1mm  共 {len(Y_SEARCH)} 点")
    print(f"  每点重复: {N_TRIALS} 次")
    print(f"  输出    : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"               : "step1_Y_v2",
            "SUCCESS_DX_m"       : SUCCESS_DX,
            "Z_OFFSET_m"         : Z_OFFSET_M,
            "MU_ROLLER_S_BASE"   : MU_ROLLER_S_BASE,
            "MU_RAIL_S"          : MU_RAIL_S,
            "MU_RAIL_D"          : MU_RAIL_D,
            "PITCH_BASE"         : PITCH_BASE,
            "Yaw_fixed_deg"      : 0.0,
            "YAW_LOCK_THRESHOLD" : YAW_LOCK_THRESHOLD,
            "YAW_LOCK_DURATION_s": YAW_LOCK_DURATION,
            "Y_range_mm"         : [round(Y_SEARCH[0]*1000,1),
                                    round(Y_SEARCH[-1]*1000,1)],
            "Y_step_mm"          : 1.0,
            "N_TRIALS"           : N_TRIALS,
        },
        "trials": [], "summary": [], "safe_range_mm": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for y in Y_SEARCH:
        y_mm = round(float(y)*1000, 1)
        successes = 0
        locks = 0
        stucks = 0
        for t in range(N_TRIALS):
            rec = await run_one_trial(
                sh, y=float(y), yaw_deg=0.0,
                pitch_deg=PITCH_BASE, z_offset=Z_OFFSET_M,
                mu_roller_s=MU_ROLLER_S_BASE,
                tag=f"Y={y_mm:+.1f}mm #{t+1}")
            rec["Y_mm"] = y_mm
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
            elif rec["result"] == "rail_lock":
                locks += 1
            elif rec["result"] == "stuck":
                stucks += 1

        # ★ 成功率只看纯 success，rail_lock 当失败
        rate = successes / N_TRIALS
        rates.append(rate)

        # 状态标记更详细
        if rate >= 0.8:
            s = "✓"
        elif locks >= N_TRIALS / 2:
            s = "🔒lock"
        elif stucks >= N_TRIALS / 2:
            s = "✗stuck"
        else:
            s = "△mix"
        print(f"  Y={y_mm:+6.1f}mm → 成功率 {rate:.0%}  "
              f"(success={successes} lock={locks} stuck={stucks}) {s}")

        log["summary"].append({
            "Y_mm"       : y_mm,
            "rate"       : round(rate, 3),
            "n_success"  : successes,
            "n_rail_lock": locks,
            "n_stuck"    : stucks,
        })
        save()

    values = [round(y*1000,1) for y in Y_SEARCH]
    log["safe_range_mm"] = extract_safe_range(values, rates, 0.8)
    save()

    print("\n" + "═" * 62)
    if log["safe_range_mm"]:
        print(f"  ✓ Y 安全区间: {log['safe_range_mm']} mm")
        print(f"    （严格定义：success 率 ≥ 80%，rail_lock 已当作失败）")
    else:
        print("  ⚠ 未找到 Y 安全区间")
        # 统计锁死占比，给出诊断
        total_trials = len(log["trials"])
        total_locks = sum(1 for t in log["trials"] if t["result"] == "rail_lock")
        if total_locks > total_trials * 0.3:
            print(f"    诊断: {total_locks}/{total_trials} 次触发 rail_lock,"
                  f" 说明当前参数下易接触导轨")
            print(f"    建议: 检查导轨间距 / 降低 μ_rail / 缩小 Y 搜索范围")
    print(f"  ✓ 已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
