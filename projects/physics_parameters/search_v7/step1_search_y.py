"""
step1_search_y.py  ── Isaac Sim 5.1 · Script Editor（完全自包含）
═══════════════════════════════════════════════════════════════════
Step 1: 固定 μ=0.060、Yaw=0°、Pitch=4°，只扫描 Y 偏心
  • Y: -50mm ~ +50mm，步进 1mm（共 101 点）
  • 抬高: z_offset = 5mm，自然掉落避免初始穿透干涉
  • 判定: x方向位移 ≥ 0.30m → success
  • 输出: projects/physics_parameters/results/safe_pose_y.json
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
import asyncio
import numpy as np

# 导入 scipy 用于四元数和欧拉角转换
try:
    from scipy.spatial.transform import Rotation
except ImportError:
    raise RuntimeError("缺少 scipy 库。请在 Isaac Sim Python 环境下执行: pip install scipy")

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

PAL_X0_ACTUAL = 0.045
PAL_Z0_ACTUAL = 1.254
PAL_Y0_CENTER = 0
PITCH_BASE    = 4.0
PALLET_PATH   = "/World/Pallet/CollisionProxy"

PHYSICS_DT        = 1.0 / 60.0
SIM_DURATION      = 8.0
STUCK_SPEED       = 0.005
STUCK_DURATION    = 3.0
POSE_SETTLE_STEPS = 5

# ★ 成功判定：位移 ≥ 0.30m（覆盖被挡板/限位器顶停的情况）
SUCCESS_DX = 0.30
Z_OFFSET_M = 0.00124  # 抬高 5mm

RESULT_DIR = str(_PHYSICS_ROOT / "results")


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
    """
    使用 Isaac Sim 原生 RigidPrim API 强制写入位姿与速度。
    这会同步修改 USD 与 PhysX 引擎内部缓存，避免位姿重置失效。
    """
    pos = np.array([float(x), float(y), float(z)])
    
    rot_x = Gf.Rotation(Gf.Vec3d(1, 0, 0), float(roll_deg))
    rot_y = Gf.Rotation(Gf.Vec3d(0, 1, 0), float(pitch_deg))
    rot_z = Gf.Rotation(Gf.Vec3d(0, 0, 1), float(yaw_deg))
    rot = rot_x * rot_y * rot_z
    
    q = rot.GetQuat()
    # Isaac Sim 的 set_world_pose 接受 [w, x, y, z] 格式
    quat_wxyz = np.array([q.GetReal(), q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2]])
    
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

async def run_one_trial(sh, *, y, yaw_deg, pitch_deg=None,
                        z_offset=0.0, mu_roller_s=None, tag=""):
    if pitch_deg is None:
        pitch_deg = PITCH_BASE
    if mu_roller_s is None:
        mu_roller_s = MU_ROLLER_S_BASE

    if sh.timeline.is_playing():
        sh.timeline.stop()

    set_roller_mu(sh, mu_roller_s)
    # 注入两侧导轨高摩擦
    rail_mat_path = "/World/Mat/RailFriction"
    rail_mat = sh.stage.GetPrimAtPath(rail_mat_path)
    if not rail_mat.IsValid():
        UsdShade.Material.Define(sh.stage, rail_mat_path)
        rail_mat = sh.stage.GetPrimAtPath(rail_mat_path)
        UsdPhysics.MaterialAPI.Apply(rail_mat)
        # 将高摩擦材质绑定给 4 根导轨
        material = UsdShade.Material(rail_mat)
        for r_path in [
            "/World/ConveyorGroup/Rail_Left_forward", "/World/ConveyorGroup/Rail_Right_forward",
            "/World/ConveyorGroup/Rail_Left_backward", "/World/ConveyorGroup/Rail_Right_backward"
        ]:
            p = sh.stage.GetPrimAtPath(r_path)
            if p.IsValid():
                if not p.HasAPI(UsdShade.MaterialBindingAPI):
                    UsdShade.MaterialBindingAPI.Apply(p)
                UsdShade.MaterialBindingAPI(p).Bind(material)
                
    rm = UsdPhysics.MaterialAPI.Get(sh.stage, rail_mat.GetPath())
    if rm:
        rm.GetStaticFrictionAttr().Set(0.6)  # 注入导轨侧向高静摩擦
        rm.GetDynamicFrictionAttr().Set(0.5) # 注入导轨侧向高动摩擦
    # done

    await sh.world.reset_async()

    set_pallet_pose(sh,
        x=PAL_X0_ACTUAL, y=PAL_Y0_CENTER + y, z=PAL_Z0_ACTUAL + z_offset,
        roll_deg=0.0, pitch_deg=pitch_deg, yaw_deg=yaw_deg)

    # 5 帧自然掉落与接触解算
    for _ in range(POSE_SETTLE_STEPS):
        await sh.app.next_update_async()

    # 读取 Settle 后的实际位姿
    pos0, rot0 = sh.pallet_prim.get_world_pose()
    start_x = float(pos0[0])
    
    # 解析真实 Yaw 角 (rot0 格式为 [w, x, y, z]，scipy 接受 [x, y, z, w])
    r = Rotation.from_quat([rot0[1], rot0[2], rot0[3], rot0[0]])
    euler_deg = r.as_euler('xyz', degrees=True)
    actual_yaw = float(euler_deg[2])
    
    if abs(actual_yaw) > 1.0:
        print(f"      [警告] 初始稳定后 Yaw 角发生偏移: {actual_yaw:.2f}° (Y 偏心可能导致干涉或单侧滑动)")

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

    if result == "timeout" and max_x - start_x >= SUCCESS_DX:
        result = "success"

    sh.timeline.stop()

    dx_mm = (max_x - start_x) * 1000
    print(f"      [{tag}] dx={dx_mm:.1f}mm → {result}  "
          f"(Yaw_settled={actual_yaw:.1f}°, v={final_v:.5f}, t={duration_used:.1f}s)")

    return {
        "result"         : result,
        "settle_yaw_deg" : round(actual_yaw, 2),
        "final_x"        : round(final_x, 5),
        "max_x"          : round(max_x, 5),
        "dx_mm"          : round(dx_mm, 2),
        "final_v"        : round(final_v, 5),
        "duration"       : round(duration_used, 2),
        "start_x"        : round(start_x, 5),
    }

# ═══════════════════════════════════════════════════════════════
# Step 1 参数 & 流程
# ═══════════════════════════════════════════════════════════════

Y_SEARCH  = np.arange(-0.025, 0.025 + 0.00001, 0.001)  # 搜索范围
N_TRIALS  = 3
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_y.json")

async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 1] Y 偏心扫描")
    print(f"  ★ 抬高测试: Z 轴已加 {Z_OFFSET_M*1000:.1f}mm 初始偏移")
    print("  ★ 判定: 位移 ≥ 300mm → success")
    print("  ★ 只扫描 Y，μ/Yaw/Pitch 全部固定")
    print("═" * 62)
    print(f"  Y 范围: {Y_SEARCH[0]*1000:.0f} ~ {Y_SEARCH[-1]*1000:.0f} mm"
          f"  步进 1mm  共 {len(Y_SEARCH)} 点")
    print(f"  每点重复: {N_TRIALS} 次")
    print(f"  输出: {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step": "step1_Y", 
            "SUCCESS_DX_m": SUCCESS_DX,
            "Z_OFFSET_m": Z_OFFSET_M,
            "MU_ROLLER_S_BASE": MU_ROLLER_S_BASE,
            "PITCH_BASE": PITCH_BASE, 
            "Yaw_fixed_deg": 0.0,
            "Y_range_mm": [round(Y_SEARCH[0]*1000,1), round(Y_SEARCH[-1]*1000,1)],
            "Y_step_mm": 1.0, 
            "N_TRIALS": N_TRIALS,
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
        for t in range(N_TRIALS):
            rec = await run_one_trial(sh, y=float(y), yaw_deg=0.0,
                pitch_deg=PITCH_BASE, z_offset=Z_OFFSET_M, 
                mu_roller_s=MU_ROLLER_S_BASE, tag=f"Y={y_mm}mm #{t+1}")
            rec["Y_mm"] = y_mm
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
        
        rate = successes / N_TRIALS
        rates.append(rate)
        s = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  Y={y_mm:6.1f}mm → 成功率 {rate:.0%} {s}")
        log["summary"].append({"Y_mm": y_mm, "rate": round(rate,3)})
        save()

    values = [round(y*1000,1) for y in Y_SEARCH]
    log["safe_range_mm"] = extract_safe_range(values, rates, 0.8)
    save()

    print("\n" + "═" * 62)
    if log["safe_range_mm"]:
        print(f"  ✓ Y 安全区间: {log['safe_range_mm']} mm")
    else:
        print("  ⚠ 未找到 Y 安全区间")
    print(f"  ✓ 已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())