import omni.usd
from pxr import Gf, UsdGeom, Usd

# 获取当前选中的物体
ctx = omni.usd.get_context()
selected_paths = ctx.get_selection().get_selected_prim_paths()

if selected_paths:
    stage = ctx.get_stage()
    for path in selected_paths:
        prim = stage.GetPrimAtPath(path)
        # 计算世界空间包围盒
        bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
        bbox = bbox_cache.ComputeWorldBound(prim)
        min_z = bbox.ComputeAlignedBox().GetMin()[2]
        
        # 将物体向上平移 -min_z，使其底部刚好落在 Z=0
        xform_api = UsdGeom.XformCommonAPI(prim)
        t, r, s, p, ro = xform_api.GetXformAttributes()
        new_translation = Gf.Vec3d(t[0], t[1], t[2] - min_z)
        xform_api.SetTranslate(new_translation)
        print(f"对齐完成：{path} 底部已平移至地面。")
else:
    print("请先在 Stage 中选中需要对齐的物体！")
