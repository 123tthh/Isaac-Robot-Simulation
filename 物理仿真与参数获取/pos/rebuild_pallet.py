import math
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# ★ 可调参数
# ════════════════════════════════════════
PALLET_SCALE = 0.9     # ★ 缩放比例

# 原始尺寸（mm→m）
PALLET_Y_ORIG = 0.265   # 横向（正对入口）
PALLET_X_ORIG = 0.185   # 纵向（沿坡滑动）
PALLET_H_ORIG = 0.1  # 高度

PALLET_MASS   = 1.0     # ★ 质量kg
PAL_Y0        = 0.000   # ★ 横向偏心(m)，0=居中，正值向右

# 缩放后实际尺寸
PALLET_Y = PALLET_Y_ORIG * PALLET_SCALE
PALLET_X = PALLET_X_ORIG * PALLET_SCALE
PALLET_H = PALLET_H_ORIG * PALLET_SCALE

# ════════════════════════════════════════
# 场景参数（与主场景保持一致，勿改）
# ════════════════════════════════════════
BASE_H    = 0.800
ROLLER_Z0 = BASE_H + 0.492
AR        = math.radians(4.0)

def z_top(x):
    return ROLLER_Z0 + 0.010 - x * math.sin(AR)

# 初始位置
PAL_X0 = PALLET_X / 2
PAL_Z0 = z_top(PAL_X0) + PALLET_H / 2 + 0.002  # 离滚轮顶2mm

# ════════════════════════════════════════
# 塑料材质
# ════════════════════════════════════════
if not stage.GetPrimAtPath("/World/Mat").IsValid():
    UsdGeom.Xform.Define(stage, "/World/Mat")

pmat_path = "/World/Mat/Pallet"
if stage.GetPrimAtPath(pmat_path).IsValid():
    stage.RemovePrim(pmat_path)

UsdShade.Material.Define(stage, pmat_path)
pm = stage.GetPrimAtPath(pmat_path)
UsdPhysics.MaterialAPI.Apply(pm).CreateStaticFrictionAttr().Set(0.40)
UsdPhysics.MaterialAPI.Apply(pm).CreateDynamicFrictionAttr().Set(0.30)
PhysxSchema.PhysxMaterialAPI.Apply(pm).CreateFrictionCombineModeAttr().Set("multiply")
pallet_mat = UsdShade.Material(pm)

# ════════════════════════════════════════
# 清除旧托盘，重建
# ════════════════════════════════════════
for p in ["/World/Pallet/CollisionProxy", "/World/Pallet"]:
    if stage.GetPrimAtPath(p).IsValid():
        stage.RemovePrim(p)

UsdGeom.Xform.Define(stage, "/World/Pallet")

proxy = UsdGeom.Cube.Define(stage, "/World/Pallet/CollisionProxy")
proxy.GetSizeAttr().Set(1.0)
pp = proxy.GetPrim()

xf = UsdGeom.XformCommonAPI(pp)
xf.SetTranslate(Gf.Vec3d(PAL_X0, PAL_Y0, PAL_Z0))
xf.SetScale(Gf.Vec3f(PALLET_X, PALLET_Y, PALLET_H))
xf.SetRotate(Gf.Vec3f(-4.0, 0.0, 0.0),
             UsdGeom.XformCommonAPI.RotationOrderXYZ)

rb = UsdPhysics.RigidBodyAPI.Apply(pp)
rb.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
UsdPhysics.MassAPI.Apply(pp).CreateMassAttr().Set(PALLET_MASS)
UsdPhysics.CollisionAPI.Apply(pp)
UsdShade.MaterialBindingAPI.Apply(pp).Bind(pallet_mat)

# ════════════════════════════════════════
# 输出确认
# ════════════════════════════════════════
FWD_W = 0.268
BWD_W = 0.268

print(f"""
✓ 托盘重建完成（方案A：仅缩小托盘）

  缩放比例 : {PALLET_SCALE} （原尺寸×{PALLET_SCALE}）
  原始尺寸 : {PALLET_Y_ORIG*1000:.0f}×{PALLET_X_ORIG*1000:.0f}×{PALLET_H_ORIG*1000:.0f} mm
  缩放尺寸 : {PALLET_Y*1000:.1f}(横)×{PALLET_X*1000:.1f}(纵)×{PALLET_H*1000:.1f}(高) mm
  材质     : 塑料 μs=0.40 μd=0.30
  质量     : {PALLET_MASS} kg
  初始位置 : ({PAL_X0:.3f}, {PAL_Y0:.3f}, {PAL_Z0:.3f}) m

间隙分析（缩放后）:
  前段进料口间隙 : ±{(FWD_W-PALLET_Y)*500:.1f} mm/侧
  后段导轨间隙   : ±{(BWD_W-PALLET_Y)*500:.1f} mm/侧

★ 调参指引:
  PALLET_SCALE = 1.0  → 原始265×185mm
  PALLET_SCALE = 0.9  → 238×167mm
  PALLET_SCALE = 0.8  → 212×148mm
  PALLET_SCALE = 0.7  → 185×130mm（当前）

★ 偏心测试（确认下滑正常后）:
  PAL_Y0 = 0.000  → 居中
  PAL_Y0 = 0.010  → 10mm偏心
  PAL_Y0 = 0.020  → 20mm偏心，观察是否卡住
""")