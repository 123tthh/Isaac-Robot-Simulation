from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════
# 直接从属性读取，不用BBoxCache
# ════════════════════════════════════
bc   = stage.GetPrimAtPath("/World/Base_Cube")
eq   = stage.GetPrimAtPath("/World/equipment")
cg   = stage.GetPrimAtPath("/World/ConveyorGroup")
pal  = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")

bc_s = bc.GetAttribute("xformOp:scale").Get()
bc_t = bc.GetAttribute("xformOp:translate").Get()
eq_t = eq.GetAttribute("xformOp:translate").Get()

BC_HEIGHT   = bc_s[2]          # 0.6m
BC_HALF     = BC_HEIGHT / 2    # 0.3m
BC_BOTTOM   = bc_t[2] - BC_HALF  # 当前底面Z
BC_TOP      = bc_t[2] + BC_HALF  # 当前顶面Z

print(f"当前 Base_Cube 底面Z = {BC_BOTTOM:.4f}m（应为0）")
print(f"当前 Base_Cube 顶面Z = {BC_TOP:.4f}m")
print(f"当前 equipment Z    = {eq_t[2]:.4f}m")

# ════════════════════════════════════
# Step 1: Base_Cube 底面贴地
# ════════════════════════════════════
new_bc_z = BC_HALF   # = 0.3，底面恰好在Z=0
bc.GetAttribute("xformOp:translate").Set(
    Gf.Vec3d(bc_t[0], bc_t[1], new_bc_z)
)
NEW_BC_TOP = new_bc_z + BC_HALF   # = 0.6m
print(f"\n✓ Base_Cube 修复:")
print(f"  translate Z: {bc_t[2]:.4f} → {new_bc_z:.4f}")
print(f"  底面Z = 0.000m ✓")
print(f"  顶面Z = {NEW_BC_TOP:.4f}m")

# ════════════════════════════════════
# Step 2: equipment 下移对齐Base_Cube顶面
# ════════════════════════════════════
EQ_TARGET_Z  = NEW_BC_TOP        # equipment底面目标 = Base_Cube顶面
EQ_DELTA     = eq_t[2] - EQ_TARGET_Z
new_eq_z     = EQ_TARGET_Z

eq.GetAttribute("xformOp:translate").Set(
    Gf.Vec3d(eq_t[0], eq_t[1], new_eq_z)
)
print(f"\n✓ equipment 修复:")
print(f"  translate Z: {eq_t[2]:.4f} → {new_eq_z:.4f}")
print(f"  下移量 = {EQ_DELTA*1000:.1f}mm")

# ════════════════════════════════════
# Step 3: ConveyorGroup 同步下移
# 原BASE_H=0.8，实际Base_Cube高=0.6，差0.2m
# ════════════════════════════════════
SCRIPT_BASE_H = 0.800
HEIGHT_DIFF   = SCRIPT_BASE_H + BC_HEIGHT  # = 0.2m

cg_t = cg.GetAttribute("xformOp:translate").Get()
if cg_t is None:
    cg_t = Gf.Vec3d(0, 0, 0)

new_cg_z = cg_t[2] - HEIGHT_DIFF
cg.GetAttribute("xformOp:translate").Set(
    Gf.Vec3d(cg_t[0], cg_t[1], new_cg_z)
)

# 验证：Roller_00_L世界Z
r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
r00_t = r00.GetAttribute("xformOp:translate").Get()
roller_world_z = r00_t[2] + new_cg_z

print(f"\n✓ ConveyorGroup 修复:")
print(f"  translate Z: {cg_t[2]:.4f} → {new_cg_z:.4f}")
print(f"  Roller_00_L 世界Z = {roller_world_z:.4f}m")
print(f"  期望 = {BC_HEIGHT + 0.492:.4f}m")

# ════════════════════════════════════
# Step 4: Pallet 同步下移
# ════════════════════════════════════
if pal.IsValid():
    pal_t = pal.GetAttribute("xformOp:translate").Get()
    if pal_t:
        new_pal_z = pal_t[2] - HEIGHT_DIFF
        pal.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(pal_t[0], pal_t[1], new_pal_z)
        )
        print(f"\n✓ Pallet 修复:")
        print(f"  translate Z: {pal_t[2]:.4f} → {new_pal_z:.4f}")

# ════════════════════════════════════
# Step 5: mesh_ 碰撞关闭（不需要与Base_Cube接触）
# ════════════════════════════════════
mesh = stage.GetPrimAtPath("/World/equipment/node_/mesh_")
if mesh.IsValid():
    col = UsdPhysics.CollisionAPI.Get(stage, mesh.GetPath())
    if col:
        col.GetCollisionEnabledAttr().Set(False)
        print(f"\n✓ mesh_ 碰撞已关闭（静态机架不需要与Base_Cube交互）")

# ════════════════════════════════════
# 结果汇总
# ════════════════════════════════════
print(f"""
══ 修复后预期 ══
GroundPlane Z       : 0.000m
Base_Cube 底面Z     : 0.000m ✓
Base_Cube 顶面Z     : {NEW_BC_TOP:.3f}m
equipment 底面目标  : {EQ_TARGET_Z:.3f}m（需视口确认）
Roller_00_L 世界Z   : {roller_world_z:.3f}m
Roller_00_L 顶面Z   : {roller_world_z+0.010:.3f}m

★ 请在视口里目视确认：
  □ Base_Cube 底面贴红色地面
  □ equipment 底面贴 Base_Cube 顶面
  □ 如有偏差在UI微调 equipment translate Z
""")