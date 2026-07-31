import math
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# 工具函数
# ════════════════════════════════════════
def remove_if_exists(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)

def make_material(path, sf, df):
    remove_if_exists(path)
    UsdShade.Material.Define(stage, path)
    p = stage.GetPrimAtPath(path)
    UsdPhysics.MaterialAPI.Apply(p).CreateStaticFrictionAttr().Set(sf)
    UsdPhysics.MaterialAPI.Apply(p).CreateDynamicFrictionAttr().Set(df)
    PhysxSchema.PhysxMaterialAPI.Apply(p).CreateFrictionCombineModeAttr().Set("multiply")
    return UsdShade.Material(p)

def make_cube(path, sx, sy, sz, tx, ty, tz, rx=0.0, ry=0.0, rz=0.0):
    """sx/sy/sz=实际尺寸(m), tx/ty/tz=中心坐标, rx/ry/rz=欧拉角(度)"""
    remove_if_exists(path)
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(1.0)
    xf = UsdGeom.XformCommonAPI(c.GetPrim())
    xf.SetTranslate(Gf.Vec3d(tx, ty, tz))
    xf.SetScale(Gf.Vec3f(sx, sy, sz))
    if rx or ry or rz:
        xf.SetRotate(Gf.Vec3f(rx, ry, rz))
    return c.GetPrim()

def add_col(prim, mat=None):
    UsdPhysics.CollisionAPI.Apply(prim)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)

# ════════════════════════════════════════
# 参数
# ════════════════════════════════════════
BASE_H   = 0.800          # Base_Cube高度

ROLLER_R = 0.010          # 滚轮半径 10mm
ROLLER_L = 0.102          # 滚轮长度 102mm
ROLLER_N = 9              # 每侧数量
ROLLER_P = 0.025          # 轴心间距 25mm
ROLLER_G = 0.104          # 左右面间距 104mm
ROLLER_Y = ROLLER_G/2 + ROLLER_L/2   # 轴心Y绝对值 = 0.103m
ROLLER_Z0 = BASE_H + 0.492            # 第一列轴心Z = 1.292m

ANG = 4.0
AR  = math.radians(ANG)

FWD_L    = 0.228          # 前段长度
FWD_W    = 0.268          # 前段末端导轨内侧宽
TILT_DEG = 5.3
TILT_R   = math.radians(TILT_DEG)
INLET_W  = FWD_W + 2*FWD_L*math.tan(TILT_R)    # 进料口宽 ≈ 0.310m
RFWD_L   = FWD_L / math.cos(TILT_R)             # 前段导轨实长 ≈ 0.229m
RAIL_T   = 0.002          # 导轨厚度
RAIL_H   = 0.050          # 导轨高度

BWD_L  = 0.296            # 后段长度
BWD_W  = 0.268            # 后段宽度
SURF_T = 0.010            # 后段接触面厚度

# 托盘：265mm横向进入导轨，185mm纵向沿坡滑动
PALLET_X = 0.185
PALLET_Y = 0.265
PALLET_H = 0.040

def z_top(x):
    """X处滚轮顶面Z高度"""
    return ROLLER_Z0 + ROLLER_R - x * math.sin(AR)

# ════════════════════════════════════════
# Step 0: 重力
# ════════════════════════════════════════
scene = UsdPhysics.Scene.Get(stage, "/World/PhysicsScene")
scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.GetGravityMagnitudeAttr().Set(9.81)
print("✓ 重力: (0,0,-1) 9.81m/s²")

# ════════════════════════════════════════
# Step 1: Base_Cube（底部贴地）
# ════════════════════════════════════════
bc = make_cube("/World/Base_Cube",
    0.60, 0.60, BASE_H,
    0.00, 0.00, BASE_H/2)   # 中心Z=0.4m，底部Z=0
add_col(bc)
print(f"✓ Base_Cube: 0.6×0.6×{BASE_H}m | Z=[0, {BASE_H}]")

# ════════════════════════════════════════
# Step 2: equipment位置 + mesh碰撞
# ════════════════════════════════════════
eq = stage.GetPrimAtPath("/World/equipment")
if eq.IsValid():
    UsdGeom.XformCommonAPI(eq).SetTranslate(Gf.Vec3d(0.0, 0.0, BASE_H))
    mesh = stage.GetPrimAtPath("/World/equipment/node_/mesh_")
    if mesh.IsValid():
        add_col(mesh)
        PhysxSchema.PhysxCollisionAPI.Apply(mesh)
    print(f"✓ equipment: 底部Z={BASE_H}m，mesh碰撞已设置")

# ════════════════════════════════════════
# Step 3: GroundPlane碰撞
# ════════════════════════════════════════
for gp in ["/World/GroundPlane/CollisionPlane",
           "/World/GroundPlane/CollisionMesh"]:
    p = stage.GetPrimAtPath(gp)
    if p.IsValid() and not UsdPhysics.CollisionAPI.Get(stage, p.GetPath()):
        UsdPhysics.CollisionAPI.Apply(p)
print("✓ GroundPlane碰撞确认")

# ════════════════════════════════════════
# Step 4: 摩擦材质
# ════════════════════════════════════════
if not stage.GetPrimAtPath("/World/Mat").IsValid():
    UsdGeom.Xform.Define(stage, "/World/Mat")
roller_mat = make_material("/World/Mat/Roller",  0.06, 0.04)  # 等效低摩擦
rail_mat   = make_material("/World/Mat/Rail",    0.20, 0.15)  # 导轨
surf_mat   = make_material("/World/Mat/Surface", 0.06, 0.04)  # 后段接触面
pallet_mat = make_material("/World/Mat/Pallet",  0.50, 0.40)  # 托盘
print("✓ 摩擦材质创建完成")

# ════════════════════════════════════════
# Step 5: 清空Conveyor
# ════════════════════════════════════════
if not stage.GetPrimAtPath("/World/Conveyor").IsValid():
    UsdGeom.Xform.Define(stage, "/World/Conveyor")
else:
    for child in list(stage.GetPrimAtPath("/World/Conveyor").GetChildren()):
        stage.RemovePrim(child.GetPath())
print("✓ Conveyor已清空，开始重建")

# ════════════════════════════════════════
# Step 6: 18个滚轮（9列×左右）
# ════════════════════════════════════════
for i in range(ROLLER_N):
    x = i * ROLLER_P
    z = ROLLER_Z0 - x * math.sin(AR)
    for side, ys in [("L", -1), ("R", +1)]:
        path = f"/World/Conveyor/Roller_{i:02d}_{side}"
        cyl = UsdGeom.Cylinder.Define(stage, path)
        cyl.GetRadiusAttr().Set(ROLLER_R)
        cyl.GetHeightAttr().Set(ROLLER_L)
        cyl.GetAxisAttr().Set("Y")
        UsdGeom.XformCommonAPI(cyl.GetPrim()).SetTranslate(
            Gf.Vec3d(x, ys * ROLLER_Y, z))
        add_col(cyl.GetPrim(), roller_mat)

print(f"✓ 滚轮: {ROLLER_N*2}个 | X=0~{(ROLLER_N-1)*ROLLER_P*1000:.0f}mm"
      f" | 轴心Y=±{ROLLER_Y*1000:.1f}mm")

# ════════════════════════════════════════
# Step 7: 前段导轨（内收5.3°）
# ════════════════════════════════════════
fwd_xc   = FWD_L / 2
fwd_y_in = INLET_W/2 - fwd_xc * math.tan(TILT_R)  # 内侧面Y均值
fwd_yc   = fwd_y_in + RAIL_T/2                     # 导轨中心Y
fwd_zc   = z_top(fwd_xc) + RAIL_H/2

p = make_cube("/World/Conveyor/Rail_Left_forward",
    RFWD_L, RAIL_T, RAIL_H,
    fwd_xc, -fwd_yc, fwd_zc,
    0.0, 0.0, +TILT_DEG)    # +5.3°绕Z：左轨随X增大向中心靠近
add_col(p, rail_mat)

p = make_cube("/World/Conveyor/Rail_Right_forward",
    RFWD_L, RAIL_T, RAIL_H,
    fwd_xc, +fwd_yc, fwd_zc,
    0.0, 0.0, -TILT_DEG)    # -5.3°绕Z：右轨镜像
add_col(p, rail_mat)
print(f"✓ 前段导轨: {RFWD_L*1000:.1f}mm"
      f" | 进料口{INLET_W*1000:.1f}mm→{FWD_W*1000:.0f}mm")

# ════════════════════════════════════════
# Step 8: 后段导轨（垂直，无内收角）
# ════════════════════════════════════════
bwd_xc = FWD_L + BWD_L/2
bwd_yc = BWD_W/2 + RAIL_T/2
bwd_zc = z_top(bwd_xc) + RAIL_H/2

p = make_cube("/World/Conveyor/Rail_Left_backward",
    BWD_L, RAIL_T, RAIL_H,
    bwd_xc, -bwd_yc, bwd_zc)
add_col(p, rail_mat)

p = make_cube("/World/Conveyor/Rail_Right_backward",
    BWD_L, RAIL_T, RAIL_H,
    bwd_xc, +bwd_yc, bwd_zc)
add_col(p, rail_mat)
print(f"✓ 后段导轨: {BWD_L*1000:.0f}mm | 内侧间距{BWD_W*1000:.0f}mm")

# ════════════════════════════════════════
# Step 9: 后段接触面（倾斜薄Cube，衔接滚轮顶面）
# ════════════════════════════════════════
# +4°绕Y轴使顶面与4°坡道平行
bwd_surf_zc = z_top(bwd_xc) - SURF_T/2

p = make_cube("/World/Conveyor/Backward_Surface",
    BWD_L, BWD_W, SURF_T,
    bwd_xc, 0.0, bwd_surf_zc,
    0.0, +ANG, 0.0)   # +4°绕Y轴
add_col(p, surf_mat)
print(f"✓ 后段接触面: {BWD_L*1000:.0f}×{BWD_W*1000:.0f}mm"
      f" | Z_center={bwd_surf_zc:.3f}m")

# ════════════════════════════════════════
# Step 10: 托盘 CollisionProxy（动态刚体）
# ════════════════════════════════════════
if not stage.GetPrimAtPath("/World/Pallet").IsValid():
    UsdGeom.Xform.Define(stage, "/World/Pallet")
remove_if_exists("/World/Pallet/CollisionProxy")

# 初始位置：进料口居中，滚轮顶面上方5mm
px = PALLET_X / 2                           # X中心=0.0925m
pz = z_top(px) + PALLET_H/2 + 0.005        # 离滚轮顶5mm避免初始穿模

p = make_cube("/World/Pallet/CollisionProxy",
    PALLET_X, PALLET_Y, PALLET_H,
    px, 0.0, pz)
add_col(p, pallet_mat)
rb = UsdPhysics.RigidBodyAPI.Apply(p)
rb.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
UsdPhysics.MassAPI.Apply(p).CreateMassAttr().Set(3.0)
print(f"✓ 托盘: X={PALLET_X*1000:.0f}×Y={PALLET_Y*1000:.0f}×Z={PALLET_H*1000:.0f}mm"
      f" | 初始位置({px:.3f}, 0, {pz:.3f})")

# ════════════════════════════════════════
# 验证摘要
# ════════════════════════════════════════
mu_eff = 0.06 * 0.40
tan_a  = math.tan(AR)
print(f"""
══ 物理验证 ══
倾角 tan({ANG}°)    = {tan_a:.4f}
等效μ(R×P)        = {mu_eff:.4f}
下滑条件          : {'✓ 满足（μ < tanθ）' if mu_eff < tan_a else '⚠ 不满足，降低roller_mat'}

══ 关键间隙（卡住分析）══
进料口侧向间隙     : ±{(INLET_W-PALLET_Y)*500:.1f}mm/侧（宽松）
后段侧向间隙       : ±{(BWD_W-PALLET_Y)*500:.1f}mm/侧（极紧，卡住高危区）
滚轮顶面Z(X=0)    : {z_top(0):.3f}m
后段末端Z(X={FWD_L+BWD_L:.3f}) : {z_top(FWD_L+BWD_L):.3f}m
总高度差           : {(z_top(0)-z_top(FWD_L+BWD_L))*1000:.1f}mm

✅ 构建完成，按Play测试
""")
