from pxr import UsdGeom, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

eq = stage.GetPrimAtPath("/World/equipment")
mesh = stage.GetPrimAtPath("/World/equipment/node_/mesh_")

# 当前变换
xf = UsdGeom.XformCommonAPI(eq)
print("=== equipment当前变换 ===")
t, r, s, pvt, ro = xf.GetXformVectors(0)
print(f"Translate : {t}")
print(f"Rotate    : {r}")
print(f"Scale     : {s}")

# Bounding Box（实际占据空间）
bc = UsdGeom.BBoxCache(0, ["default"])
bbox = bc.ComputeWorldBound(eq)
rng = bbox.GetRange()
mn, mx = rng.GetMin(), rng.GetMax()
size = mx - mn
print(f"\n=== equipment BoundingBox ===")
print(f"Min : ({mn[0]:.3f}, {mn[1]:.3f}, {mn[2]:.3f})")
print(f"Max : ({mx[0]:.3f}, {mx[1]:.3f}, {mx[2]:.3f})")
print(f"Size: X={size[0]:.3f}m  Y={size[1]:.3f}m  Z={size[2]:.3f}m")
