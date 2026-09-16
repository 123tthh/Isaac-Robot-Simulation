"""
几何约束与摩擦耦合导致的“楔形锁死”（wedging/jamming）

在真实物理中，当一个较宽的物体（托盘）在一个狭窄的通道（导轨）中滑动时，如果导轨侧面的滑动摩擦系数较大，
一旦托盘发生微小的偏航（Yaw）导致对角线接触导轨，高摩擦力会瞬间产生一个巨大的反向锁死力矩。托盘直接卡死”，在之前的仿真中，
因为侧面导轨显式设置摩擦过小（引擎使用了默认的极低摩擦或完美弹性碰撞），托盘像抹了油一样滑蹭着导轨转了过去，从而得到了“假成功” 推导锁死力矩

"""
import omni.usd
from pxr import UsdPhysics, UsdShade, Gf, Sdf

def check_and_update_rail_friction(target_mu_s=0.6, target_mu_k=0.5):
    stage = omni.usd.get_context().get_stage()
    
    rails = [
        "/World/ConveyorGroup/Rail_Left_forward",
        "/World/ConveyorGroup/Rail_Right_forward",
        "/World/ConveyorGroup/Rail_Left_backward",
        "/World/ConveyorGroup/Rail_Right_backward"
    ]

    print("══════════════════════════════════════════════════════════════")
    print("  导轨侧面摩擦力体检与修复")
    print("══════════════════════════════════════════════════════════════")

    # 1. 在场景中创建或获取专门的导轨摩擦材质
    rail_mat_path = "/World/Mat/RailFriction"
    mat_prim = stage.GetPrimAtPath(rail_mat_path)
    
    if not mat_prim.IsValid():
        # 如果不存在，创建一个新的材质节点
        UsdShade.Material.Define(stage, rail_mat_path)
        mat_prim = stage.GetPrimAtPath(rail_mat_path)
        UsdPhysics.MaterialAPI.Apply(mat_prim)
        print(f"  [+] 创建了新的导轨物理材质: {rail_mat_path}")
    else:
        print(f"  [*] 找到现有的导轨物理材质: {rail_mat_path}")

    # 2. 设置高摩擦系数 (根据你的现实观察，金属/塑料干摩擦通常在 0.5~0.7 之间)
    mat_api = UsdPhysics.MaterialAPI(mat_prim)
    mat_api.CreateStaticFrictionAttr().Set(target_mu_s)
    mat_api.CreateDynamicFrictionAttr().Set(target_mu_k)
    print(f"  ✓ 导轨材质摩擦力已更新 -> 静摩擦: {target_mu_s}, 动摩擦: {target_mu_k}")

    # 3. 检查并强制绑定到 4 根导轨上
    for path in rails:
        prim = stage.GetPrimAtPath(path)
        if not prim.IsValid():
            print(f"  ✗ 找不到轨道: {path}")
            continue

        binding_api = UsdShade.MaterialBindingAPI(prim)
        current_mat = binding_api.GetDirectBinding().GetMaterialPath()
        
        status = f"原材质: {current_mat if current_mat else '无 (物理引擎默认空材质)'}"
        
        # 强制绑定新材质
        material = UsdShade.Material(mat_prim)
        binding_api.Bind(material)
        print(f"  -> {prim.GetName()}: {status} | 现已强制绑定高摩擦材质！")

    print("══════════════════════════════════════════════════════════════")
    print("修复完成！请务必按 Ctrl + S 保存当前 USD 场景。")

# 你可以根据实际情况微调这里的数值，0.6 是一个很容易触发“卡死(Wedging)”的高阻力值
check_and_update_rail_friction(target_mu_s=0.6, target_mu_k=0.5)