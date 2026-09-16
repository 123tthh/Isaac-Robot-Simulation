import math
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# Part 1: 诊断 - 读取所有关键Z坐标
# ════════════════════════════════════════
print("=== 当前Z坐标诊断 ===\n")

def get_bbox(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"  [MISSING] {path}")
        return None, None
    bc   = UsdGeom.BBoxCache(0, ["default"])
    bbox = bc.ComputeWorldBound(prim)
    rng  = bbox.GetRange()
    return rng.GetMin()[2], rng.GetMax()[2]

def get_translate(path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None
    t = prim.GetAttribute("xformOp:translate").Get()
    return t

# GroundPlane
gp_min, gp_max = get_bbox("/World/GroundPlane")
print(f"GroundPlane     : Z_min={gp_min:.4f}  Z_max={gp_max:.4f}")

# Base_Cube
bc_min, bc_max = get_bbox("/World/Base_Cube")
bc_t = get_translate("/World/Base_Cube")
bc_s = stage.GetPrimAtPath("/World/Base_Cube").GetAttribute("xformOp:scale").Get()
print(f"Base_Cube       : Z_min={bc_min:.4f}  Z_max={bc_max:.4f}")
print(f"  translate     : {bc_t}")
print(f"  scale         : {bc_s}")

# equipment
eq_min, eq_max = get_bbox("/World/equipment")
eq_t = get_translate("/World/equipment")
print(f"equipment       : Z_min={eq_min:.4f}  Z_max={eq_max:.4f}")
print(f"  translate     : {eq_t}")

# ConveyorGroup
cg_t = get_translate("/World/ConveyorGroup")
print(f"ConveyorGroup   : translate={cg_t}")

# Roller_00_L 轴心
r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
if r00.IsValid():
    rt = r00.GetAttribute("xformOp:translate").Get()
    print(f"Roller_00_L     : translate={rt}")

print(f"\n=== 穿模分析 ===")
if bc_max and eq_min:
    overlap = bc_max - eq_min
    print(f"Base_Cube顶面Z  : {bc_max:.4f}")
    print(f"equipment底面Z  : {eq_min:.4f}")
    print(f"穿插量          : {overlap*1000:.1f}mm "
          f"({'穿模!' if overlap > 0.001 else '正常'})")

if bc_min:
    print(f"Base_Cube底面Z  : {bc_min:.4f} "
          f"({'未贴地!' if abs(bc_min) > 0.001 else '贴地OK'})")