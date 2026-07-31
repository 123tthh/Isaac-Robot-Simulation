import omni.usd
from pxr import Usd, UsdShade

def check_material_properties():
    stage = omni.usd.get_context().get_stage()
    if not stage:
        print("未找到打开的 Stage！")
        return

    # 定义你需要重点排查的物体路径
    targets = [
        "/World/Pallet/CollisionProxy",              # 托盘
        "/World/ConveyorGroup/Roller_00_L",          # 滚轮示例 (左右各一)
        "/World/ConveyorGroup/Roller_00_R",
        "/World/ConveyorGroup/Roller_08_L",
        "/World/ConveyorGroup/Roller_08_R",
        "/World/ConveyorGroup/Backward_Surface",     # 后部平面
        
        "/World/ConveyorGroup/Rail_Right_forward",   # 轨道侧面 (左右各一)
        "/World/ConveyorGroup/Rail_Left_forward"
        "/World/ConveyorGroup/Rail_Right_backward",
        "/World/ConveyorGroup/Rail_Left_backward"
    ]

    print("══════════════════════════════════════════════════════════════")
    print("  接触面物理属性与摩擦组合模式排查报告")
    print("══════════════════════════════════════════════════════════════")

    mode_map = {
        "average": "Average (平均/加性) - [PhysX默认]",
        "min": "Min (取两面最小值)",
        "multiply": "Multiply (乘性模拟) - [真实物理常推荐]",
        "max": "Max (取两面最大值)"
    }

    for path in targets:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  ✗ 找不到物体: {path} (如果重命名了请忽略)")
            continue
        
        print(f"\n▶ 检查物体: {prim.GetName()}")

        # 1. 查找绑定的物理材质
        mat_path = None
        binding_api = UsdShade.MaterialBindingAPI(prim)
        if binding_api:
            direct_binding = binding_api.GetDirectBinding()
            if direct_binding.GetMaterialPath():
                mat_path = direct_binding.GetMaterialPath()
        
        if not mat_path:
            print("  ⚠ 未直接绑定物理材质，引擎将使用默认的空材质属性。")
            mat_prim = prim  # 尝试直接从物体上读属性
        else:
            print(f"  ✓ 绑定的物理材质: {mat_path}")
            mat_prim = stage.GetPrimAtPath(mat_path)
        
        # 2. 读取标准摩擦力属性
        s_fric = mat_prim.GetAttribute("physics:staticFriction").Get()
        d_fric = mat_prim.GetAttribute("physics:dynamicFriction").Get()
        
        print(f"    • 静摩擦 (Static) : {s_fric if s_fric is not None else '未设置 (默认)'}")
        print(f"    • 动摩擦 (Dynamic): {d_fric if d_fric is not None else '未设置 (默认)'}")

        # 3. 读取组合模式 (Combine Mode)
        fric_mode = mat_prim.GetAttribute("physxMaterial:frictionCombineMode").Get()
        rest_mode = mat_prim.GetAttribute("physxMaterial:restitutionCombineMode").Get()

        fric_str = mode_map.get(str(fric_mode), str(fric_mode)) if fric_mode else "未显式设置 (采用引擎默认 Average)"
        print(f"    • 摩擦组合模式    : {fric_str}")

        # 4. 读取接触面刚度与阻尼 (Compliant Contact)
        stiff = mat_prim.GetAttribute("physxMaterial:contactStiffness").Get()
        damp = mat_prim.GetAttribute("physxMaterial:contactDamping").Get()

        if stiff is not None or damp is not None:
            print(f"    • 接触面刚度系数  : {stiff}")
            print(f"    • 接触面阻尼系数  : {damp}")
            print("      (说明: 该接触面启用了柔性软接触模拟)")
        else:
            print("    • 接触面刚度      : 绝对刚体 (Rigid) - [无弹性形变穿透容差]")

check_material_properties()