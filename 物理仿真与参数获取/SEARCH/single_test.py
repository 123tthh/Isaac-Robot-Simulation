import asyncio, os, math, json
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd, omni.timeline, omni.kit.app

stage   = omni.usd.get_context().get_stage()
SAVE_MD = os.path.expanduser(
    "~/Desktop/IssacLab_arena_assets/res.md")
os.makedirs(os.path.dirname(SAVE_MD), exist_ok=True)

lines = []
def log(msg=""):
    print(msg)
    lines.append(str(msg))
def flush():
    with open(SAVE_MD, "w") as f:
        f.write("\n".join(lines))

# ════════════════════════════════
# 参数
# ════════════════════════════════
PAL_X0    = 0.045
PAL_Y0    = 0.00253
PAL_Z0    = 1.26751
SUCCESS_X = 0.450
SIM_STEPS = 300      # 5秒 × 60fps
PALLET_PATH = "/World/Pallet/CollisionProxy"

async def main():
    log("=== Isaac Sim 5.1 正确仿真方式 ===")
    flush()

    # ── Step1: 初始化SimulationContext ──
    from isaacsim.core.api.simulation_context import SimulationContext

    sim = SimulationContext.instance()
    if sim is None:
        sim = SimulationContext(
            physics_dt=1/60.0,
            rendering_dt=1/60.0,
            stage_units_in_meters=1.0
        )
        log("✓ SimulationContext 新建")
    else:
        log("✓ SimulationContext 已存在")

    # ── Step2: 初始化并重置 ──
    await sim.initialize_simulation_context_async()
    log("✓ initialize_simulation_context_async 完成")

    # ── Step3: 设置USD位置（Stop状态下写入）──
    proxy = stage.GetPrimAtPath(PALLET_PATH)
    rb    = UsdPhysics.RigidBodyAPI.Get(stage, proxy.GetPath())

    def reset_pallet(y=0.0, yaw=0.0, pitch=4.0):
        proxy.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(PAL_X0, PAL_Y0 + y, PAL_Z0)
        )
        proxy.GetAttribute("xformOp:rotateXYZ").Set(
            Gf.Vec3f(0.0, float(pitch), float(yaw))
        )
        if rb:
            rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
            rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))

    reset_pallet()
    await sim.reset_async()   # ← 重置后自动Play
    log("✓ reset_async 完成（仿真已启动）")
    flush()

    # ── Step4: 用next_update_async推进帧 ──
    # 这是Script Editor里正确的推进方式
    app = omni.kit.app.get_app()
    log(f"\n开始步进 {SIM_STEPS} 帧...")
    flush()

    for i in range(SIM_STEPS):
        await app.next_update_async()   # ← 推进一帧（含物理步进）

        # 每60帧读一次
        if (i+1) % 60 == 0:
            t = proxy.GetAttribute("xformOp:translate").Get()
            v = rb.GetVelocityAttr().Get() if rb else None
            spd = (v[0]**2+v[1]**2+v[2]**2)**0.5 if v else 0
            log(f"  {(i+1)//60}s | X={t[0]:.4f} Z={t[2]:.4f}"
                f" | 速度={spd:.5f}m/s")
            flush()

    # ── Step5: 读取最终结果 ──
    t1  = proxy.GetAttribute("xformOp:translate").Get()
    v1  = rb.GetVelocityAttr().Get() if rb else None
    dx  = t1[0] - PAL_X0
    spd = (v1[0]**2+v1[1]**2+v1[2]**2)**0.5 if v1 else 0

    if t1[0] >= SUCCESS_X:
        result = 'success'
    elif spd < 0.005:
        result = 'stuck'
    else:
        result = 'timeout'

    log(f"\n最终位置: ({t1[0]:.5f}, {t1[1]:.5f}, {t1[2]:.5f})")
    log(f"X总位移 : {dx*1000:.2f}mm")
    log(f"最终速度: {spd:.5f}m/s")
    log(f"判定结果: {result}")

    # ── Step6: 停止 ──
    await sim.stop_async()
    log("\n✓ 仿真已停止")
    log("=== 完成 ===")
    flush()
    print(f"✅ 结果写入: {SAVE_MD}")

# 启动
asyncio.ensure_future(main())