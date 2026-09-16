from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

cube_prim = stage.GetPrimAtPath("/World/Cube")
cube_geo  = UsdGeom.Cube(cube_prim)

# 读取当前scale，算出半高
scale = cube_prim.GetAttribute("xformOp:scale").Get()
if scale:
    half_height = scale[2]   # Z方向半边长
else:
    # Cube默认size=2，scale=1时半高=1
    # 如果没有scale属性，用GetSizeAttr
    size = cube_geo.GetSizeAttr().Get()
    half_height = size / 2 if size else 1.0

# 底面对齐地面：Z = half_height
UsdGeom.XformCommonAPI(cube_prim).SetTranslate(
    Gf.Vec3d(0.0, 0.0, half_height)
)

print(f"Cube半高: {half_height}")
print(f"Cube中心Z已设为: {half_height}（底面贴地面Z=0）")