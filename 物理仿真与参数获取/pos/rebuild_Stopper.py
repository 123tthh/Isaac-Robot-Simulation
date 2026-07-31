import math
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ★ 可调参数
BASE_H    = 0.800
ROLLER_Z0 = BASE_H + 0.492
FWD_L     = 0.228
BWD_L     = 0.296
BWD_W     = 0.268
AR        = math.radians(4.0)

STOP_T    = 0.005   # ★ 挡板厚度 5mm
STOP_H    = 0.080   # ★ 挡板高度 80mm
STOP_W    = BWD_W   # ★ 挡板宽度=后段宽度

def z_top(x):
    return ROLLER_Z0 + 0.010 - x * math.sin(AR)

STOP_X = FWD_L + BWD_L
STOP_Z = z_top(STOP_X) + STOP_H / 2

# 删除旧挡板
if stage.GetPrimAtPath("/World/Conveyor/Stopper").IsValid():
    stage.RemovePrim("/World/Conveyor/Stopper")

stopper = UsdGeom.Cube.Define(stage, "/World/Conveyor/Stopper")
stopper.GetSizeAttr().Set(1.0)
xf = UsdGeom.XformCommonAPI(stopper.GetPrim())
xf.SetTranslate(Gf.Vec3d(STOP_X + STOP_T/2, 0.0, STOP_Z))
xf.SetScale(Gf.Vec3f(STOP_T, STOP_W, STOP_H))

UsdPhysics.CollisionAPI.Apply(stopper.GetPrim())

print(f"✓ 后挡板:")
print(f"  位置X : {STOP_X:.3f}m（后段末端）")
print(f"  宽度  : {STOP_W*1000:.0f}mm")
print(f"  高度  : {STOP_H*1000:.0f}mm")
print(f"  中心Z : {STOP_Z:.3f}m")