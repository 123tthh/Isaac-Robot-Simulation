import math
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# Step 1: 读取滚轮实际世界Z（不依赖公式）
# ════════════════════════════════════════
r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
cg  = stage.GetPrimAtPath("/World/ConveyorGroup")

if not r00.IsValid():
    print("⚠ Roller_00_L不存在，检查路径")
else:
    r00_local_z = r00.GetAttribute("xformOp:translate").Get()[2]
    cg_z        = cg.GetAttribute("xformOp:translate").Get()[2] \
                  if cg.IsValid() else 0.0

    # 滚轮轴心实际世界Z
    ROLLER_AXIS_Z  = r00_local_z + cg_z
    ROLLER_R       = 0.010
    # 滚轮顶面实际世界Z（X=0处）
    ROLLER_TOP_Z0  = ROLLER_AXIS_Z + ROLLER_R

    AR = math.radians(4.0)

    print(f"Roller_00_L 局部Z    : {r00_local_z:.4f}m")
    print(f"ConveyorGroup Z      : {cg_z:.4f}m")
    print(f"滚轮轴心世界Z        : {ROLLER_AXIS_Z:.4f}m")
    print(f"滚轮顶面Z(X=0)       : {ROLLER_TOP_Z0:.4f}m")

    # ════════════════════════════════════════
    # Step 2: 计算新托盘尺寸
    # 原始尺寸：Y=265mm X=185mm Z=100mm
    # 缩放比例：265:257.816（入口宽度约束）
    # ════════════════════════════════════════
    INLET_W     = 0.25782   # 实际入口宽度
    ORIG_Y      = 0.265
    SCALE       = INLET_W / ORIG_Y   # 缩放比例

    PALLET_Y    = ORIG_Y  * SCALE    # = 0.257816m（贴合入口）
    PALLET_X    = 0.185   * SCALE    # = 0.17998m
    PALLET_H    = 0.100   * SCALE    # = 0.09729m

    print(f"\n缩放比例  : {SCALE:.5f}  ({INLET_W*1000:.3f}/{ORIG_Y*1000:.0f}mm)")
    print(f"PALLET_Y  : {PALLET_Y*1000:.3f}mm")
    print(f"PALLET_X  : {PALLET_X*1000:.3f}mm")
    print(f"PALLET_H  : {PALLET_H*1000:.3f}mm")

    # ════════════════════════════════════════
    # Step 3: 计算托盘初始位置
    # 3/4长度在滚轮上：PAL_X0 = PALLET_X/4
    # 托盘下表面 = 滚轮顶面
    # 托盘中心Z  = 滚轮顶面Z + PALLET_H/2
    # ════════════════════════════════════════
    PAL_X0 = PALLET_X / 4   # 3/4在滚轮上

    # 该X处滚轮顶面Z（考虑4°坡度，绕Y轴）
    roller_top_at_x = ROLLER_TOP_Z0 - PAL_X0 * math.sin(AR)

    # 正确公式：下表面贴滚轮顶面
    # 下表面Z = 中心Z - PALLET_H/2 = roller_top_at_x
    # → 中心Z = roller_top_at_x + PALLET_H/2
    PAL_Z0 = roller_top_at_x + PALLET_H / 2

    PAL_Y0 = 0.000   # ★ 横向偏心，居中

    print(f"\nPAL_X0            : {PAL_X0*1000:.2f}mm")
    print(f"该X处滚轮顶面Z    : {roller_top_at_x:.4f}m")
    print(f"托盘中心Z         : {PAL_Z0:.4f}m")
    print(f"托盘下表面Z       : {PAL_Z0 - PALLET_H/2:.4f}m（应={roller_top_at_x:.4f}m）")
    print(f"贴合误差          : {abs((PAL_Z0-PALLET_H/2)-roller_top_at_x)*1000:.3f}mm")

    # ════════════════════════════════════════
    # Step 4: 重建托盘
    # ════════════════════════════════════════
    # 清除旧托盘
    for p in ["/World/Pallet/CollisionProxy", "/World/Pallet"]:
        if stage.GetPrimAtPath(p).IsValid():
            stage.RemovePrim(p)

    UsdGeom.Xform.Define(stage, "/World/Pallet")

    # 塑料材质
    pmat_path = "/World/Mat/Pallet"
    if stage.GetPrimAtPath(pmat_path).IsValid():
        stage.RemovePrim(pmat_path)
    UsdShade.Material.Define(stage, pmat_path)
    pm = stage.GetPrimAtPath(pmat_path)
    UsdPhysics.MaterialAPI.Apply(pm).CreateStaticFrictionAttr().Set(0.40)
    UsdPhysics.MaterialAPI.Apply(pm).CreateDynamicFrictionAttr().Set(0.30)
    PhysxSchema.PhysxMaterialAPI.Apply(pm).CreateFrictionCombineModeAttr().Set("multiply")
    pallet_mat = UsdShade.Material(pm)

    # CollisionProxy
    proxy = UsdGeom.Cube.Define(stage, "/World/Pallet/CollisionProxy")
    proxy.GetSizeAttr().Set(1.0)
    pp = proxy.GetPrim()

    xf = UsdGeom.XformCommonAPI(pp)
    xf.SetTranslate(Gf.Vec3d(PAL_X0, PAL_Y0, PAL_Z0))
    xf.SetScale(Gf.Vec3f(PALLET_X, PALLET_Y, PALLET_H))
    # 绕Y轴旋转-4°（与场景坡度一致）
    xf.SetRotate(Gf.Vec3f(0.0, 4.0, 0.0),
                 UsdGeom.XformCommonAPI.RotationOrderXYZ)

    rb = UsdPhysics.RigidBodyAPI.Apply(pp)
    rb.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
    rb.CreateAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
    UsdPhysics.MassAPI.Apply(pp).CreateMassAttr().Set(3.0)
    UsdPhysics.CollisionAPI.Apply(pp)
    UsdShade.MaterialBindingAPI.Apply(pp).Bind(pallet_mat)

    print(f"""
✓ 托盘重建完成
  尺寸(XYZ)  : {PALLET_X*1000:.2f} × {PALLET_Y*1000:.3f} × {PALLET_H*1000:.2f} mm
  缩放比例   : {SCALE:.5f}
  旋转       : (0, -4°, 0) 绕Y轴，平行坡面
  中心位置   : ({PAL_X0:.4f}, {PAL_Y0:.4f}, {PAL_Z0:.4f})
  下表面Z    : {PAL_Z0-PALLET_H/2:.4f}m
  滚轮顶面Z  : {roller_top_at_x:.4f}m
  3/4在滚轮上: {PALLET_X*0.75*1000:.1f}mm
  1/4悬空    : {PALLET_X*0.25*1000:.1f}mm

★ 按Play确认：
  □ 托盘下表面贴滚轮顶面
  □ 托盘能缓慢加速下滑
  □ 调整PAL_Y0做偏心测试
""")