"""
single_test.py  (Isaac Sim 5.1 · Script Editor 专用)
────────────────────────────────────────────────────────────────
参考：
  • docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/robots_simulation.html
  • docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/util_snippets.html
  • docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_isaac_snippets.html
      （simulation_get_data.py 的 Script Editor 版本）
  • docs.isaacsim.omniverse.nvidia.com/5.1.0/reference_material/sim_performance_optimization_handbook.html

关键修正：
  1. 用 World（自带 SimulationContext），而不是裸的 SimulationContext
  2. World.instance() 若已存在则 clear_instance() 再建，避免 initialize 卡住
  3. 用 SingleRigidPrim 从 PhysX tensors 直接读位姿/速度
     （仿真期间 USD 属性不同步，读 rb.GetVelocityAttr() 得到的永远是 0）
  4. 只初始化一次 SimulationContext，之后每次试验只做
       timeline.stop → 改 USD → world.reset_async → 循环 next_update_async
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
import asyncio
import math

import omni.usd
import omni.kit.app
import omni.timeline
from pxr import UsdPhysics, Gf

# ═════════════════════════════════════════════════════════════
# 参数
# ═════════════════════════════════════════════════════════════
PALLET_PATH = "/World/Pallet/CollisionProxy"

PAL_X0    = 0.045
PAL_Y0    = 0.00253
PAL_Z0    = 1.26751
PITCH_DEG = 4.0
SUCCESS_X = 0.450

SIM_DURATION = 5.0         # 秒
PHYSICS_DT   = 1.0 / 60.0  # 物理步长
STUCK_SPEED  = 0.005       # m/s

SAVE_MD = str(_PHYSICS_ROOT / "results" / "single_test.md")
os.makedirs(os.path.dirname(SAVE_MD), exist_ok=True)

# ═════════════════════════════════════════════════════════════
# 日志工具
# ═════════════════════════════════════════════════════════════
_lines = []
def log(msg=""):
    print(msg)
    _lines.append(str(msg))
def flush():
    with open(SAVE_MD, "w") as f:
        f.write("\n".join(_lines))

# ═════════════════════════════════════════════════════════════
# 主流程
# ═════════════════════════════════════════════════════════════
async def main():
    log("=" * 60)
    log("Isaac Sim 5.1 · 托盘下滑单次测试（官方推荐模式）")
    log("=" * 60)
    flush()

    # ── Step 1: 拿到 Stage 与托盘 prim ────────────────────────
    stage = omni.usd.get_context().get_stage()
    proxy = stage.GetPrimAtPath(PALLET_PATH)
    if not proxy.IsValid():
        log(f"✗ 找不到 {PALLET_PATH}，终止"); flush(); return
    rb_usd = UsdPhysics.RigidBodyAPI.Get(stage, proxy.GetPath())
    if not rb_usd:
        log(f"✗ {PALLET_PATH} 没有 RigidBodyAPI，终止"); flush(); return
    log(f"✓ 找到托盘: {PALLET_PATH}")

    # ── Step 2: 建 World（带 SimulationContext）────────────────
    # 参考 robots_simulation.html 第8-12行的标准写法
    from isaacsim.core.api.world import World

    if World.instance() is not None:
        log("• 已有 World 实例，先清理")
        World.instance().clear_instance()

    world = World(
        physics_dt=PHYSICS_DT,
        rendering_dt=PHYSICS_DT,
        stage_units_in_meters=1.0,
    )
    log(f"✓ 新建 World  (dt={PHYSICS_DT*1000:.2f}ms)")

    # initialize_simulation_context_async 必须 await，否则不会真正初始化
    await world.initialize_simulation_context_async()
    log("✓ initialize_simulation_context_async 完成")
    flush()

    # ── Step 3: 用 SingleRigidPrim 包住托盘，从 PhysX 直接读状态 ──
    # 这是关键修正 —— 仿真期间 rb.GetVelocityAttr() 永远是 0，
    # 必须用 SingleRigidPrim 才能读到真实的 PhysX 状态
    try:
        from isaacsim.core.prims import SingleRigidPrim
        pallet = SingleRigidPrim(prim_path=PALLET_PATH, name="pallet")
        log("✓ SingleRigidPrim 包装完成 (isaacsim.core.prims)")
    except ImportError:
        # 老接口兜底（4.x 时期的路径）
        from omni.isaac.core.prims import RigidPrim as SingleRigidPrim
        pallet = SingleRigidPrim(prim_path=PALLET_PATH, name="pallet")
        log("✓ SingleRigidPrim 包装完成 (omni.isaac.core.prims 兼容路径)")

    # ── Step 4: 设置初始位姿（stop 状态下写入 USD）──────────────
    timeline = omni.timeline.get_timeline_interface()
    if timeline.is_playing():
        timeline.stop()

    proxy.GetAttribute("xformOp:translate").Set(
        Gf.Vec3d(PAL_X0, PAL_Y0, PAL_Z0)
    )
    rot_attr = proxy.GetAttribute("xformOp:rotateXYZ")
    if rot_attr:
        rot_attr.Set(Gf.Vec3f(0.0, float(PITCH_DEG), 0.0))
    log(f"✓ 初始位姿: ({PAL_X0}, {PAL_Y0}, {PAL_Z0}), pitch={PITCH_DEG}°")
    flush()

    # ── Step 5: reset_async 启动仿真 ──────────────────────────
    # reset_async 内部：stop → set_initial_state → play → 走1帧
    await world.reset_async()
    log("✓ world.reset_async 完成（仿真已启动）")
    flush()

    # ── Step 6: 用 next_update_async 推进帧，每帧从 PhysX 读状态 ──
    n_steps = int(SIM_DURATION / PHYSICS_DT)
    log(f"\n开始推进 {n_steps} 帧 ({SIM_DURATION:.1f}秒)...")
    log(f"{'步数':>6} {'时间(s)':>8} {'X(m)':>10} {'Y(m)':>10} {'Z(m)':>10} {'速度(m/s)':>12}")
    flush()

    app = omni.kit.app.get_app()
    for i in range(n_steps):
        await app.next_update_async()

        if (i + 1) % 30 == 0:   # 每 0.5 秒打印一次
            # ★ 关键：从 SingleRigidPrim（PhysX tensor）读，不读 USD
            pos, _quat = pallet.get_world_pose()
            lin_vel = pallet.get_linear_velocity()
            speed = float((lin_vel[0]**2 + lin_vel[1]**2 + lin_vel[2]**2) ** 0.5)
            t_now = (i + 1) * PHYSICS_DT
            log(f"{i+1:>6d} {t_now:>8.2f}"
                f" {float(pos[0]):>10.4f} {float(pos[1]):>10.4f} {float(pos[2]):>10.4f}"
                f" {speed:>12.5f}")
            flush()

    # ── Step 7: 读取最终状态并判定 ──────────────────────────
    pos_end, _ = pallet.get_world_pose()
    vel_end    = pallet.get_linear_velocity()
    x_end = float(pos_end[0])
    y_end = float(pos_end[1])
    z_end = float(pos_end[2])
    speed_end = float((vel_end[0]**2 + vel_end[1]**2 + vel_end[2]**2) ** 0.5)
    dx = x_end - PAL_X0

    if x_end >= SUCCESS_X:
        result = "success (下滑到末端)"
    elif z_end < PAL_Z0 - 0.15:
        result = "fallen (掉落)"
    elif speed_end < STUCK_SPEED:
        result = "stuck (卡住)"
    else:
        result = "timeout (仍在移动)"

    log("\n" + "─" * 60)
    log(f"最终位置 : ({x_end:.5f}, {y_end:.5f}, {z_end:.5f})")
    log(f"X 位移   : {dx*1000:.2f} mm")
    log(f"最终速度 : {speed_end:.5f} m/s")
    log(f"判定     : {result}")
    log("─" * 60)

    # ── Step 8: 停止仿真 ─────────────────────────────────────
    timeline.stop()
    log("\n✓ 仿真已停止")
    log(f"✓ 结果写入: {SAVE_MD}")
    log("=" * 60)
    flush()

# 启动（Script Editor 里这一句放在最外层）
asyncio.ensure_future(main())
