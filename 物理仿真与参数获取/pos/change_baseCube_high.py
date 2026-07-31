from pxr import UsdGeom, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# ★ 参数
# ════════════════════════════════════════
# 从诊断读取的实际值
BC_SCALE_Z   = 0.6     # Base_Cube scale Z（实际高度）
BC_HALF_H    = BC_SCALE_Z / 2   # = 0.3m

# 脚本设定的BASE_H vs 实际Base_Cube高度的差
BASE_H_SCRIPT  = 0.800  # 脚本里设定的值
BASE_H_ACTUAL  = BC_SCALE_Z     # 实际高度 0.6m
HEIGHT_DIFF    = BASE_H_SCRIPT - BASE_H_ACTUAL  # = 0.2m 需要下移

print("=== 修复计划 ===")
print(f"Base_Cube实际高度 : {BASE_H_ACTUAL}m")
print(f"脚本设定BASE_H    : {BASE_H_SCRIPT}m")
print(f"ConveyorGroup下移 : {HEIGHT_DIFF}m")

# ════════════════════════════════════════
# Step 1: Base_Cube 底面贴地
# 底面Z=0 → translate Z = 半高 = 0.3
# ════════════════════════════════════════
bc_prim = stage.GetPrimAtPath("/World/Base_Cube")
if bc_prim.IsValid():
    old_t = bc_prim.GetAttribute("xformOp:translate").Get()
    new_t = Gf.Vec3d(old_t[0], old_t[1], BC_HALF_H)
    bc_prim.GetAttribute("xformOp:translate").Set(new_t)
    print(f"\n✓ Base_Cube:")
    print(f"  translate Z: {old_t[2]:.4f} → {BC_HALF_H:.4f}")
    print(f"  底面Z = {BC_HALF_H - BC_HALF_H:.4f}m（贴地）")
    print(f"  顶面Z = {BC_HALF_H * 2:.4f}m")
else:
    print("⚠ Base_Cube不存在")

# ════════════════════════════════════════
# Step 2: ConveyorGroup 下移HEIGHT_DIFF
# 补偿BASE_H与实际高度的差值
# ════════════════════════════════════════
cg_prim = stage.GetPrimAtPath("/World/ConveyorGroup")
if cg_prim.IsValid():
    old_t = cg_prim.GetAttribute("xformOp:translate").Get()
    if old_t is None:
        old_t = Gf.Vec3d(0, 0, 0)
    new_z = old_t[2] - HEIGHT_DIFF
    cg_prim.GetAttribute("xformOp:translate").Set(
        Gf.Vec3d(old_t[0], old_t[1], new_z)
    )
    # 验证Roller_00_L新位置
    r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
    if r00.IsValid():
        rt = r00.GetAttribute("xformOp:translate").Get()
        new_roller_z = rt[2] + new_z  # 世界Z = 局部Z + group偏移
        print(f"\n✓ ConveyorGroup:")
        print(f"  translate Z: {old_t[2]:.4f} → {new_z:.4f}")
        print(f"  Roller_00_L 世界Z: {1.292:.4f} → {new_roller_z:.4f}")
        print(f"  期望滚轮世界Z = {BASE_H_ACTUAL + 0.492:.4f}m")
else:
    print("⚠ ConveyorGroup不存在")

# ════════════════════════════════════════
# Step 3: Pallet 同步下移
# ════════════════════════════════════════
pallet_prim = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
if pallet_prim.IsValid():
    old_t = pallet_prim.GetAttribute("xformOp:translate").Get()
    if old_t:
        new_t = Gf.Vec3d(old_t[0], old_t[1], old_t[2] - HEIGHT_DIFF)
        pallet_prim.GetAttribute("xformOp:translate").Set(new_t)
        print(f"\n✓ Pallet:")
        print(f"  translate Z: {old_t[2]:.4f} → {new_t[2]:.4f}")
else:
    print("⚠ Pallet/CollisionProxy不存在，跳过")

# ════════════════════════════════════════
# Step 4: 验证结果
# ════════════════════════════════════════
print(f"""
══ 修复后预期状态 ══
GroundPlane Z        : 0.000m（不动）
Base_Cube 底面Z      : 0.000m（贴地）
Base_Cube 顶面Z      : {BASE_H_ACTUAL:.3f}m
Roller_00_L 世界Z    : {BASE_H_ACTUAL + 0.492:.3f}m
Roller_00_L 顶面Z    : {BASE_H_ACTUAL + 0.492 + 0.010:.3f}m

★ 如果equipment底面仍不在Base_Cube顶面：
  在UI里调整equipment的translate Z
  目标：equipment底面 ≈ {BASE_H_ACTUAL:.3f}m
""")