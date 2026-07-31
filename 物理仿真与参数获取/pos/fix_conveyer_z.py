from pxr import UsdGeom, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

cg  = stage.GetPrimAtPath("/World/ConveyorGroup")
r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")

cg_t  = cg.GetAttribute("xformOp:translate").Get()
r00_t = r00.GetAttribute("xformOp:translate").Get()

# Step3下移了0.2m，原路加回来
REVERT = -0.100
new_cg_z = cg_t[2] + REVERT

cg.GetAttribute("xformOp:translate").Set(
    Gf.Vec3d(cg_t[0], cg_t[1], new_cg_z)
)

roller_world_z = r00_t[2] + new_cg_z

print(f"ConveyorGroup Z: {cg_t[2]:.4f} → {new_cg_z:.4f}")
print(f"Roller_00_L 世界Z: {roller_world_z:.4f}m")
print(f"期望: {0.600 + 0.492:.4f}m")

# Pallet同步
import math
pal = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
if pal.IsValid():
    pal_t = pal.GetAttribute("xformOp:translate").Get()
    if pal_t:
        new_pal_z = pal_t[2] + REVERT
        pal.GetAttribute("xformOp:translate").Set(
            Gf.Vec3d(pal_t[0], pal_t[1], new_pal_z)
        )
        print(f"Pallet Z: {pal_t[2]:.4f} → {new_pal_z:.4f}")

print("✓ 完成")