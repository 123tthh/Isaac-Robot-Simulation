import math
from pxr import UsdGeom, UsdPhysics, UsdShade, Gf, Sdf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# ★ 唯一需要调整的参数
# ════════════════════════════════════════
CONVEYOR_Z_OFFSET = 0   # ★ 整体高度偏移(m)，正值向上，负值向下
                             #   调好后保持0，用equipment位置对齐

# ════════════════════════════════════════
# Part 1: 整合所有Conveyor零件到ConveyorGroup
# 包含：18个Roller、4个Rail、Backward_Surface、Stopper
# 不包含：equipment、GroundPlane、Pallet、Base_Cube
# ════════════════════════════════════════

GROUP_PATH = "/World/ConveyorGroup"

# 创建根节点（如已存在则保留）
if not stage.GetPrimAtPath(GROUP_PATH).IsValid():
    UsdGeom.Xform.Define(stage, GROUP_PATH)
    print(f"✓ 创建 ConveyorGroup 根节点")
else:
    print(f"✓ ConveyorGroup 已存在，更新位置")

# 设置整体偏移（调整整体高度只需改CONVEYOR_Z_OFFSET）
UsdGeom.XformCommonAPI(
    stage.GetPrimAtPath(GROUP_PATH)
).SetTranslate(Gf.Vec3d(0.0, 0.0, CONVEYOR_Z_OFFSET))

# 需要整合的子节点路径列表
conveyor_children = []

# 18个滚轮
for i in range(9):
    for side in ["L", "R"]:
        conveyor_children.append(f"/World/Conveyor/Roller_{i:02d}_{side}")

# 4个导轨
for name in ["Rail_Left_forward", "Rail_Right_forward",
             "Rail_Left_backward", "Rail_Right_backward"]:
    conveyor_children.append(f"/World/Conveyor/{name}")

# 后段接触面和挡板
conveyor_children.append("/World/Conveyor/Backward_Surface")
conveyor_children.append("/World/Conveyor/Stopper")

# 迁移到ConveyorGroup
moved   = 0
missing = 0

for src_path in conveyor_children:
    prim = stage.GetPrimAtPath(src_path)
    if not prim.IsValid():
        print(f"  [SKIP] {src_path} 不存在")
        missing += 1
        continue

    # 目标路径
    name     = src_path.split("/")[-1]
    dst_path = f"{GROUP_PATH}/{name}"

    # 读取当前世界变换（translate + scale + rotate）
    t_attr = prim.GetAttribute("xformOp:translate").Get()
    s_attr = prim.GetAttribute("xformOp:scale").Get()
    r_attr = prim.GetAttribute("xformOp:rotateXYZ").Get()

    # 获取prim类型
    prim_type = prim.GetTypeName()

    # 在ConveyorGroup下重建同类型节点
    if stage.GetPrimAtPath(dst_path).IsValid():
        stage.RemovePrim(dst_path)

    if prim_type == "Cylinder":
        new_prim = UsdGeom.Cylinder.Define(stage, dst_path)
        # 复制Cylinder属性
        src_cyl = UsdGeom.Cylinder(prim)
        new_prim.GetRadiusAttr().Set(src_cyl.GetRadiusAttr().Get())
        new_prim.GetHeightAttr().Set(src_cyl.GetHeightAttr().Get())
        new_prim.GetAxisAttr().Set(src_cyl.GetAxisAttr().Get())
        new_p = new_prim.GetPrim()
    elif prim_type == "Cube":
        new_prim = UsdGeom.Cube.Define(stage, dst_path)
        src_size = UsdGeom.Cube(prim).GetSizeAttr().Get()
        new_prim.GetSizeAttr().Set(src_size if src_size else 1.0)
        new_p = new_prim.GetPrim()
    else:
        print(f"  [SKIP] 未知类型 {prim_type}: {src_path}")
        missing += 1
        continue

    # 复制变换
    xf = UsdGeom.XformCommonAPI(new_p)
    if t_attr:
        xf.SetTranslate(Gf.Vec3d(t_attr[0], t_attr[1], t_attr[2]))
    if s_attr:
        xf.SetScale(Gf.Vec3f(s_attr[0], s_attr[1], s_attr[2]))
    if r_attr:
        xf.SetRotate(Gf.Vec3f(r_attr[0], r_attr[1], r_attr[2]),
                     UsdGeom.XformCommonAPI.RotationOrderXYZ)

    # 复制CollisionAPI
    if UsdPhysics.CollisionAPI.Get(stage, prim.GetPath()):
        UsdPhysics.CollisionAPI.Apply(new_p)

    # 复制MaterialBinding
    src_bind = UsdShade.MaterialBindingAPI(prim)
    bound    = src_bind.GetDirectBinding().GetMaterialPath()
    if bound:
        mat = UsdShade.Material(stage.GetPrimAtPath(bound))
        if mat:
            UsdShade.MaterialBindingAPI.Apply(new_p).Bind(mat)

    # 删除原节点
    stage.RemovePrim(src_path)
    moved += 1

# 删除空的旧Conveyor节点
old_conveyor = stage.GetPrimAtPath("/World/Conveyor")
if old_conveyor.IsValid():
    remaining = list(old_conveyor.GetChildren())
    if not remaining:
        stage.RemovePrim("/World/Conveyor")
        print("✓ 旧 /World/Conveyor 已清空删除")
    else:
        print(f"⚠ /World/Conveyor 仍有 {len(remaining)} 个子节点未迁移")

print(f"\n✓ ConveyorGroup 整合完成: {moved}个迁移 | {missing}个跳过")
print(f"  调整整体高度只需修改顶部 CONVEYOR_Z_OFFSET")

# ════════════════════════════════════════
# Part 2: 地面设置红色
# ════════════════════════════════════════
def set_red(prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[SKIP] {prim_path} 不存在")
        return

    mat_path = f"{prim_path}/RedMat"
    if stage.GetPrimAtPath(mat_path).IsValid():
        stage.RemovePrim(mat_path)

    mat  = UsdShade.Material.Define(stage, mat_path)
    shdr = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")
    shdr.CreateIdAttr("UsdPreviewSurface")
    shdr.CreateInput("diffuseColor",
                     Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(1.0, 0.0, 0.0))
    shdr.CreateInput("roughness",
                     Sdf.ValueTypeNames.Float).Set(0.8)

    mat.CreateSurfaceOutput().ConnectToSource(
        shdr.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)
    print(f"✓ 红色材质已绑定: {prim_path}")

# 地面相关节点
for path in ["/World/GroundPlane",
             "/World/GroundPlane/CollisionMesh",
             "/World/GroundPlane/CollisionPlane"]:
    set_red(path)

# ════════════════════════════════════════
# Part 3: Base_Cube 对齐
# 上表面贴 equipment(mesh) 底面
# 下表面贴地面 Z=0
# ════════════════════════════════════════

# 读取equipment mesh的世界坐标，找底面Z
mesh_prim = stage.GetPrimAtPath("/World/equipment/node_/mesh_")
eq_prim   = stage.GetPrimAtPath("/World/equipment")

if mesh_prim.IsValid() and eq_prim.IsValid():
    # 计算equipment世界BBox
    bc   = UsdGeom.BBoxCache(0, ["default"])
    bbox = bc.ComputeWorldBound(eq_prim)
    rng  = bbox.GetRange()
    eq_bottom_z = rng.GetMin()[2]   # equipment底面世界Z

    # Base_Cube：底面Z=0，顶面=equipment底面
    bc_height = eq_bottom_z         # Base_Cube高度
    bc_path   = "/World/Base_Cube"
    bc_prim   = stage.GetPrimAtPath(bc_path)

    if bc_prim.IsValid():
        xf = UsdGeom.XformCommonAPI(bc_prim)
        xf.SetTranslate(Gf.Vec3d(0.0, 0.0, bc_height / 2))
        xf.SetScale(Gf.Vec3f(0.60, 0.60, bc_height))
        print(f"\n✓ Base_Cube 对齐完成:")
        print(f"  底面Z = 0.000m（贴地面）")
        print(f"  顶面Z = {bc_height:.4f}m（贴equipment底面）")
        print(f"  高度  = {bc_height*1000:.1f}mm")
    else:
        print(f"⚠ Base_Cube 不存在，请先创建")
else:
    print("⚠ equipment 路径无效，Base_Cube 无法自动对齐")
    print("  请手动设置 Base_Cube 高度")

# ════════════════════════════════════════
# 使用说明
# ════════════════════════════════════════
print(f"""
══ 整体高度调整说明 ══

调整 ConveyorGroup 整体高度：
  修改脚本顶部 CONVEYOR_Z_OFFSET 重跑即可
  例：CONVEYOR_Z_OFFSET = 0.05  → 整体上移50mm

各节点职责：
  /World/ConveyorGroup   ← 所有滚轮+导轨+挡板（整体移动）
  /World/equipment       ← 机架视觉（单独调整）
  /World/Base_Cube       ← 自动贴合equipment底面
  /World/GroundPlane     ← 红色地面（固定Z=0）
  /World/Pallet          ← 托盘（单独调整）
""")