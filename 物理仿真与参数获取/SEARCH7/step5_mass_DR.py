"""
step5_mass_DR.py  ── Isaac Sim 5.1 · Script Editor (Standalone)
═══════════════════════════════════════════════════════════════════
Step 5: 质量域随机化 (Mass Domain Randomization) 扫描
        ★ 固定：Y, Yaw, Pitch, μ 为前置步骤的安全中心点
        ★ 变量：Pallet 质量在 0.7kg ~ 2.0kg 之间动态调整
        ★ 核心：每次修改质量时，严格按照 1:1 尺寸同步重算并写入转动惯量

尺寸基准: X=185mm, Y=265mm, Z=40mm
输出：~/Desktop/user_scripts/result/safe_pose_mass.json
═══════════════════════════════════════════════════════════════════
"""

import os
import sys
import json
import asyncio
import numpy as np

import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdGeom, UsdPhysics, Gf

# ═══════════════════════════════════════════════════════════════
# 共享常量 (已适配 1:1 比例)
# ═══════════════════════════════════════════════════════════════

MU_ROLLER_S_BASE = 0.060
PALLET_MU_S      = 0.400
MU_ROLLER_BASE   = MU_ROLLER_S_BASE * PALLET_MU_S

PAL_X0_ACTUAL = 0.045
PAL_Z0_ACTUAL = 1.254
PAL_Y0_CENTER = 0.0      # ★ 1:1 严格对称中心
PITCH_BASE    = 4.0
PALLET_PATH   = "/World/Pallet/CollisionProxy"

PHYSICS_DT        = 1.0 / 60.0
SIM_DURATION      = 8.0
STUCK_SPEED       = 0.005
STUCK_DURATION    = 3.0
POSE_SETTLE_STEPS = 5

SUCCESS_DX = 0.40        # ★ 1:1 比例下的成功判定位移
Z_OFFSET_M = 0.003       # 3mm 初始防穿透掉落缓冲

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


# ═══════════════════════════════════════════════════════════════
# 物理工具函数
# ═══════════════════════════════════════════════════════════════

def set_pallet_pose(sh, x, y, z, roll_deg, pitch_deg, yaw_deg):
    pos = np.array([float(x), float(y), float(z)])
    rot_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), float(roll_deg))
    rot_y = Gf.Rotation(Gf.Vec3d(0, 1, 0), float(pitch_deg))
    rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg))
    rot = rot_x * rot_y * rot_z
    q = rot.GetQuat()
    quat_wxyz = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    
    sh.pallet_prim.set_world_pose(position=pos, orientation=quat_wxyz)
    sh.pallet_prim.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
    sh.pallet_prim.set_angular_velocity(np.array([0.0, 0.0, 0.0]))


def set_roller_mu(sh, mu_s):
    mat = sh.stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid(): return False
    m = UsdPhysics.MaterialAPI.Get(sh.stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(float(mu_s))
        m.GetDynamicFrictionAttr().Set(float(mu_s) * 0.7)
    return True

def set_pallet_mass_and_inertia(sh, mass_kg):
    """★ 核心：动态更新质量，并按 1:1 尺寸同步计算并更新转动惯量"""
    mass_api = UsdPhysics.MassAPI.Get(sh.stage, PALLET_PATH)
    if not mass_api:
        print("    ⚠ 警告: Pallet 尚未挂载 MassAPI！")
        return False
        
    # 托盘物理尺寸 (单位: m)
    dim_x, dim_y, dim_z = 0.185, 0.265, 0.040
    
    # 理论公式: I = 1/12 * M * (a^2 + b^2)
    ixx = (1.0 / 12.0) * mass_kg * (dim_y**2 + dim_z**2)
    iyy = (1.0 / 12.0) * mass_kg * (dim_x**2 + dim_z**2)
    izz = (1.0 / 12.0) * mass_kg * (dim_x**2 + dim_y**2)
    
    # 写入 USD
    mass_api.GetMassAttr().Set(float(mass_kg))
    mass_api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(ixx, iyy, izz))
    return True


def extract_safe_range(values, rates, threshold=0.8):
    safe_idx = [i for i, r in enumerate(rates) if r >= threshold]
    if not safe_idx: return None
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
# 执行单次试车
# ═══════════════════════════════════════════════════════════════

async def run_one_trial(sh, *, y, yaw_deg, pitch_deg, z_offset, mu_roller_s, mass_kg, tag=""):
    if sh.timeline.is_playing():
        sh.timeline.stop()

    # 1. 停机状态下设置所有物理属性（材质与质量）
    set_roller_mu(sh, mu_roller_s)
    set_pallet_mass_and_inertia(sh, mass_kg)

    # 2. Reset 环境，让引擎重新读取新的质量和惯量
    await sh.world.reset_async()

    # 3. 强行设定坐标
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

        if max_x - start_x >= SUCCESS_DX:
            result = "success"
            break

        if v_mag < STUCK_SPEED:
            stuck_counter += 1
        else:
            stuck_counter = 0

        if stuck_counter >= stuck_frames:
            if max_x - start_x >= SUCCESS_DX:
                result = "success"
            else:
                result = "stuck"
            break

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
# Step 5 质量参数扫描
# ═══════════════════════════════════════════════════════════════

MASS_SEARCH = np.arange(0.7, 2.0 + 0.001, 0.1)  # 0.7kg ~ 2.0kg，步进 0.1kg
N_TRIALS    = 3

# 尝试读取之前步骤的结果，作为本步骤的基准参数
SAFE_POSE_INPUT = os.path.join(RESULT_DIR, "safe_pose.json")
SAVE_PATH       = os.path.join(RESULT_DIR, "safe_pose_mass.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 5] Mass 质量域随机化扫描")
    print("  ★ 固定: 最佳姿态系 (Y/Yaw/Pitch/μ) 进行受力测试")
    print("  ★ 变量: 模拟空载到满载，0.7kg ~ 2.0kg 动态修正惯量")
    print("═" * 62)

    # 尝试从 Step4 读取安全中点
    y_fixed_m   = 0.0
    yaw_fixed   = 0.0
    pitch_fixed = PITCH_BASE
    mu_fixed    = MU_ROLLER_S_BASE

    if os.path.exists(SAFE_POSE_INPUT):
        with open(SAFE_POSE_INPUT) as f:
            data = json.load(f)
            pos_cfg = data.get("final_safe_pose", {})
            
            y_range     = pos_cfg.get("position", {}).get("Y_range_mm", [0.0, 0.0])
            yaw_range   = pos_cfg.get("orientation", {}).get("Yaw_range_deg", [0.0, 0.0])
            pitch_range = pos_cfg.get("orientation", {}).get("Pitch_range_deg", [4.0, 4.0])
            mu_range    = pos_cfg.get("friction", {}).get("mu_roller_s_range")
            
            y_fixed_m   = ((y_range[0] + y_range[1]) / 2.0) / 1000.0
            yaw_fixed   = (yaw_range[0] + yaw_range[1]) / 2.0
            pitch_fixed = (pitch_range[0] + pitch_range[1]) / 2.0
            if mu_range:
                mu_fixed = (mu_range[0] + mu_range[1]) / 2.0
        print("  ✓ 已成功加载 Step4 最终的 Safe Pose 作为基准。")
    else:
        print("  ⚠ 找不到 step4 safe_pose.json，将使用默认标定常量进行扫描。")

    print(f"  基准 Y    : {y_fixed_m*1000:.2f} mm")
    print(f"  基准 Yaw  : {yaw_fixed:.2f} °")
    print(f"  基准 Pitch: {pitch_fixed:.2f} °")
    print(f"  基准 μ_s  : {mu_fixed:.4f}")
    print("-" * 62)
    print(f"  Mass 范围 : {MASS_SEARCH[0]:.1f}kg ~ {MASS_SEARCH[-1]:.1f}kg  步进0.1kg 共 {len(MASS_SEARCH)} 点")
    print(f"  每点重复  : {N_TRIALS} 次")
    print(f"  输出文件  : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step": "step5_Mass_DR",
            "Y_fixed_mm": round(y_fixed_m * 1000, 2),
            "Yaw_fixed_deg": round(yaw_fixed, 2),
            "Pitch_fixed_deg": round(pitch_fixed, 2),
            "mu_roller_s_fixed": round(mu_fixed, 4),
            "MASS_range_kg": [round(float(MASS_SEARCH[0]), 2), round(float(MASS_SEARCH[-1]), 2)],
            "Z_OFFSET_m": Z_OFFSET_M, 
            "N_TRIALS": N_TRIALS,
        },
        "trials": [], "summary": [],
        "safe_mass_kg_range": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for mass in MASS_SEARCH:
        m_kg = round(float(mass), 2)
        successes = 0
        for t in range(N_TRIALS):
            rec = await run_one_trial(sh, y=y_fixed_m,
                yaw_deg=float(yaw_fixed), pitch_deg=float(pitch_fixed),
                z_offset=Z_OFFSET_M, mu_roller_s=mu_fixed, mass_kg=m_kg,
                tag=f"M={m_kg:.1f}kg #{t+1}")
                
            rec["Mass_kg"] = m_kg
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
                
        rate = successes / N_TRIALS
        rates.append(rate)
        s = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  Mass = {m_kg:.1f} kg → 成功率 {rate:.0%} {s}")
        log["summary"].append({"Mass_kg": m_kg, "rate": round(rate,3)})
        save()

    mass_vals = [round(float(m), 2) for m in MASS_SEARCH]
    safe_mass = extract_safe_range(mass_vals, rates, 0.8)
    log["safe_mass_kg_range"] = safe_mass
    save()

    print("\n" + "═" * 62)
    if safe_mass:
        print(f"  ✓ 质量安全域 (DR Range): {safe_mass} kg")
        print("  (系统在上述质量变动下展现出极高的鲁棒性)")
    else:
        print("  ⚠ 未找到连续的质量安全区间，承载力极端敏感。")
    print(f"  ✓ 数据写入: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())