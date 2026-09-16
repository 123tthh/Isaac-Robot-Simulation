from pxr import UsdGeom, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ── 参数 ──
TARGET_SCALE = 0.001       # 按10.4:1缩放，mm→m
BASE_H       = 0.800       # Base_Cube顶面高度

eq   = stage.GetPrimAtPath("/World/equipment")
xf   = UsdGeom.XformCommonAPI(eq)

# ── 先设Scale，再算BBox ──
xf.SetScale(Gf.Vec3f(TARGET_SCALE, TARGET_SCALE, TARGET_SCALE))

# ── 计算缩放后的BoundingBox底部Z ──
from pxr import UsdGeom as UG
bc   = UG.BBoxCache(0, ["default"])
bbox = bc.ComputeWorldBound(eq)
rng  = bbox.GetRange()
mn   = rng.GetMin()
mx   = rng.GetMax()
size = mx - mn

print(f"缩放后尺寸: X={size[0]:.3f}m  Y={size[1]:.3f}m  Z={size[2]:.3f}m")
print(f"缩放后BBox Min Z = {mn[2]:.3f}m")

# ── 底部对齐Base_Cube顶面 ──
# 当前底部Z = mn[2]，需要移到BASE_H
# 平移量 = BASE_H - mn[2]
offset_z = BASE_H - mn[2]

# XY方向暂时居中（你后续手动调整）
xf.SetTranslate(Gf.Vec3d(0.0, 0.0, offset_z))

# ── 重新计算确认 ──
bc2  = UG.BBoxCache(0, ["default"])
bbox2 = bc2.ComputeWorldBound(eq)
rng2  = bbox2.GetRange()
mn2   = rng2.GetMin()
mx2   = rng2.GetMax()

print(f"\n对齐后:")
print(f"  底部Z = {mn2[2]:.3f}m（应≈{BASE_H:.3f}m）")
print(f"  顶部Z = {mx2[2]:.3f}m")
print(f"  XY范围: X=[{mn2[0]:.3f}, {mx2[0]:.3f}]  Y=[{mn2[1]:.3f}, {mx2[1]:.3f}]")
print(f"\n✓ 完成，XY位置请在UI里手动微调")
