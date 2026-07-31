import omni.usd
from pxr import UsdPhysics, Gf, UsdGeom, Usd

def setup_1to1_scene_v2():
    stage = omni.usd.get_context().get_stage()
    print("正在执行 1:1 场景重构 (版本 V2)...")

    # ─── 1. 更新托盘 (Pallet) ───────────────────────────────────
    # 尺寸: x=185, y=265, z=40 mm
    pallet_path = "/World/Pallet/CollisionProxy"
    pallet_prim = stage.GetPrimAtPath(pallet_path)
    if pallet_prim.IsValid():

        pallet_prim.GetAttribute("xformOp:scale").Set(Gf.Vec3d(0.185, 0.265, 0.040))
        
        # 更新位置: 保持 x, z 位置不变，强制 y 位置居中 (0.0)
        xf = UsdGeom.XformCommonAPI(pallet_prim)
        curr_pos = xf.GetXformVectors(Usd.TimeCode.Default())[0]
        xf.SetTranslate(Gf.Vec3d(curr_pos[0], 0.0, curr_pos[2]))
        
        # 更新物理属性: M=1.875kg, 写入计算好的转动惯量
        mass_api = UsdPhysics.MassAPI.Get(stage, pallet_path)
        if not mass_api: 
            mass_api = UsdPhysics.MassAPI.Apply(pallet_prim)
        mass_api.CreateMassAttr().Set(1.875)
        mass_api.CreateCenterOfMassAttr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        mass_api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(0.01122, 0.00560, 0.01632))
        print("  ✓ 托盘: 尺寸更新为 185x265x40mm，Y轴已归零，转动惯量已校准。")

    # ─── 2. 更新导轨 (Rails) ─────────────────────────────────────
    # 内宽 268mm -> Y = ±134mm
    # 前段导轨偏转角 5.27度
    rail_params = {
        "/World/ConveyorGroup/Rail_Left_backward":  {"y": 0.134, "yaw": 0.0},
        "/World/ConveyorGroup/Rail_Right_backward": {"y": -0.134, "yaw": 0.0},
        "/World/ConveyorGroup/Rail_Left_forward":   {"y": 0.134, "yaw": -5.27},
        "/World/ConveyorGroup/Rail_Right_forward":  {"y": -0.134, "yaw": 5.27},
    }

    for path, p in rail_params.items():
        prim = stage.GetPrimAtPath(path)
        if prim.IsValid():
            xf = UsdGeom.XformCommonAPI(prim)
            curr_pos = xf.GetXformVectors(Usd.TimeCode.Default())[0]
            # 更新 Y 位置，保持 X, Z 位置
            xf.SetTranslate(Gf.Vec3d(curr_pos[0], p["y"], curr_pos[2]))
            # 更新角度: X=0, Y=4(Pitch), Z=Yaw(5.27)
            xf.SetRotate(Gf.Vec3f(0.0, 4.0, p["yaw"]), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            
    print("  ✓ 导轨: 后段开口 268mm，前段偏转角 5.27°，Pitch 4°。")

    # ─── 3. 滚筒 (Rollers) ──────────────────────────────────────
    # 脚本不执行任何操作，严格保持其现有的 X, Y, Z 位置
    print("  ✓ 滚筒: 已跳过位置更新，保持原有微调后的轴心位置。")

    print("1:1 场景重构完成！请按 Ctrl+S 保存。")

setup_1to1_scene_v2()