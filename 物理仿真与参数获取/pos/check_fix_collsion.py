from pxr import UsdPhysics, UsdShade, PhysxSchema
import omni.usd

stage = omni.usd.get_context().get_stage()

# 确保roller_mat存在
roller_mat_path = "/World/Mat/Roller"
roller_mat_prim = stage.GetPrimAtPath(roller_mat_path)
if not roller_mat_prim.IsValid():
    print("⚠ /World/Mat/Roller 不存在，请先运行主构建脚本")
else:
    roller_mat = UsdShade.Material(roller_mat_prim)
    print("✓ RollerMat找到")

    fixed = 0
    ok    = 0

    for i in range(9):
        for side in ["L", "R"]:
            path = f"/World/Conveyor/Roller_{i:02d}_{side}"
            prim = stage.GetPrimAtPath(path)

            if not prim.IsValid():
                print(f"[MISSING] {path}")
                continue

            col = UsdPhysics.CollisionAPI.Get(stage, prim.GetPath())
            bound = UsdShade.MaterialBindingAPI(
                prim).GetDirectBinding().GetMaterialPath()

            need_col = not col
            need_mat = not bound or str(bound) != roller_mat_path

            if need_col:
                UsdPhysics.CollisionAPI.Apply(prim)
            if need_mat:
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(roller_mat)

            if need_col or need_mat:
                fixed += 1
                print(f"[FIXED] Roller_{i:02d}_{side}"
                      f" col={'补加' if need_col else 'OK'}"
                      f" mat={'补绑' if need_mat else 'OK'}")
            else:
                ok += 1

    print(f"\n✓ 完成: {ok}个正常 | {fixed}个已修复")