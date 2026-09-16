import omni.usd
from pxr import UsdGeom, Usd

# 获取当前选中的物体
ctx = omni.usd.get_context()
selected_paths = ctx.get_selection().get_selected_prim_paths()

if not selected_paths:
    print("⚠️ 请先在 Stage 中选中要测量的物体！")
else:
    stage = ctx.get_stage()
    # 获取场景的单位（通常是 1.0 = 1米）
    meters_per_unit = UsdGeom.GetStageMetersPerUnit(stage)
    
    for path in selected_paths:
        prim = stage.GetPrimAtPath(path)
        
        # 计算包含所有子层级的世界坐标包围盒
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(prim)
        
        # 如果物体无效或没有几何体
        if bbox.GetRange().IsEmpty():
            print(f"物体 {path} 没有实体体积。")
            continue
            
        aligned_box = bbox.ComputeAlignedBox()
        min_pt = aligned_box.GetMin()
        max_pt = aligned_box.GetMax()
        
        # 计算 X, Y, Z 方向的长度
        length_x = max_pt[0] - min_pt[0]
        width_y  = max_pt[1] - min_pt[1]
        height_z = max_pt[2] - min_pt[2]
        
        print("-" * 40)
        print(f"📦 目标物体: {path}")
        print(f"📏 场景单位: 1 Unit = {meters_per_unit} 米")
        print(f"➡️ X轴 (长): {length_x:.4f} Units")
        print(f"↗️ Y轴 (宽): {width_y:.4f} Units")
        print(f"⬆️ Z轴 (高): {height_z:.4f} Units")
        print("-" * 40)
