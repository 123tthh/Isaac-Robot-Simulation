from pxr import UsdPhysics, PhysxSchema
import omni.usd

stage = omni.usd.get_context().get_stage()

mat_paths = [
    "/World/Mat/Roller",
    "/World/Mat/Surface",
    "/World/Mat/Rail",
    "/World/Mat/Pallet",
]

print("=== 当前摩擦系数 ===\n")
for path in mat_paths:
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        print(f"[MISSING] {path}")
        continue

    m = UsdPhysics.MaterialAPI.Get(stage, prim.GetPath())
    px = PhysxSchema.PhysxMaterialAPI.Get(stage, prim.GetPath())

    sf = m.GetStaticFrictionAttr().Get()  if m  else "?"
    df = m.GetDynamicFrictionAttr().Get() if m  else "?"
    cm = px.GetFrictionCombineModeAttr().Get() if px else "?"

    # 计算与托盘的等效μ（multiply combine）
    if path != "/World/Mat/Pallet" and sf != "?":
        pallet = stage.GetPrimAtPath("/World/Mat/Pallet")
        pm = UsdPhysics.MaterialAPI.Get(stage, pallet.GetPath())
        pf = pm.GetStaticFrictionAttr().Get() if pm else 1.0
        mu_eff = sf * pf
    else:
        mu_eff = None

    print(f"{path.split('/')[-1]:<12}"
          f" μs={sf:.4f}  μd={df:.4f}"
          f"  combine={cm}"
          + (f"  → 等效μ={mu_eff:.4f}" if mu_eff else ""))

# tan(4°)临界值
import math
tan4 = math.tan(math.radians(4.0))
print(f"\ntan(4°) = {tan4:.4f}")
print(f"下滑条件：等效μ < {tan4:.4f}")