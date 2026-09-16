import math
from pxr import UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════════════════════
# ★ 可调整参数区（所有需要手动修改的参数在此集中）
# ════════════════════════════════════════════════════════

# ── 高度基准 ─────────────────────────────────────────
BASE_H      = 0.800   # ★ Base_Cube高度，用于与机器人手臂对齐

# ── 滚轮几何 ─────────────────────────────────────────
ROLLER_R    = 0.010   # ★ 滚轮半径 10mm
ROLLER_L    = 0.102   # ★ 滚轮轴向长度 102mm
ROLLER_N    = 9       # ★ 每侧滚轮数量
ROLLER_P    = 0.025   # ★ 相邻滚轮轴心间距 25mm
ROLLER_G    = 0.104   # ★ 左右滚轮面间距（面到面）104mm

# ── 输送带整体 ───────────────────────────────────────
ANGLE_DEG   = 4.0     # ★ 坡度角（度），进料口高于末端
ROLLER_Z0   = BASE_H + 0.492  # ★ 第一列滚轮轴心Z（机架底+492mm）

# ── 前段（含滚轮区）────────────────────────────────
FWD_L       = 0.228   # ★ 前段沿X方向长度 228mm
FWD_W       = 0.268   # ★ 前段末端导轨内侧宽 268mm
TILT_DEG    = 5.3     # ★ 前段导轨内收角 5.3°
RAIL_H      = 0.040   # ★ 导轨高度 40mm（可调）
RAIL_T      = 0.002   # ★ 导轨厚度 2mm

# ── 后段（Backward_Surface取代22个小滚轮）──────────
BWD_L       = 0.296   # ★ 后段长度 296mm
BWD_W       = 0.268   # ★ 后段宽度 268mm
SURF_T      = 0.010   # ★ 后段接触面厚度 10mm

# ── 托盘（塑料）────────────────────────────────────
PALLET_X    = 0.265   # ★ 托盘长（沿输送方向进入）265mm
PALLET_Y    = 0.185   # ★ 托盘宽 185mm
PALLET_H    = 0.040   # ★ 托盘高 40mm
PALLET_MASS = 3.0     # ★ 托盘质量kg（含零件）

# ── 摩擦系数（★ 行为标定后替换）───────────────────
# 不锈钢滚轮 vs 塑料托盘（等效滚动摩擦，需仿真标定）
MU_ROLLER_S = 0.06    # ★ 滚轮等效静摩擦
MU_ROLLER_D = 0.04    # ★ 滚轮等效动摩擦
# 不锈钢导轨 vs 塑料托盘（参考值：不锈钢-塑料0.35~0.45）
MU_RAIL_S   = 0.35    # ★ 导轨静摩擦
MU_RAIL_D   = 0.25    # ★ 导轨动摩擦
# 后段接触面（取代小滚轮，等效摩擦同前段滚轮）
MU_SURF_S   = 0.06    # ★ 后段接触面静摩擦
MU_SURF_D   = 0.04    # ★ 后段接触面动摩擦
# 塑料托盘材质属性
MU_PALLET_S = 0.40    # ★ 托盘静摩擦
MU_PALLET_D = 0.30    # ★ 托盘动摩擦

# ════════════════════════════════════════════════════════
# 派生参数（自动计算，无需手动修改）
# ════════════════════════════════════════════════════════
AR       = math.radians(ANGLE_DEG)
TILT_R   = math.radians(TILT_DEG)
ROLLER_Y = ROLLER_G/2 + ROLLER_L/2          # 滚轮轴心Y绝对值=0.103m
INLET_W  = FWD_W + 2*FWD_L*math.tan(TILT_R) # 进料口宽≈0.310m
RFWD_L   = FWD_L / math.cos(TILT_R)         # 前段导轨实长≈0.229m

def z_top(x):
    """X处滚轮/接触面顶面Z高度"""
    return ROLLER_Z0 + ROLLER_R - x * math.sin(AR)

# ════════════════════════════════════════════════════════
# 工具函数
# ════════════════════════════════════════════════════════
def remove_if_exists(path):
    if stage.GetPrimAtPath(path).IsValid():
        stage.RemovePrim(path)

def make_material(path, sf, df, combine="multiply"):
    remove_if_exists(path)
    UsdShade.Material.Define(stage, path)
    p = stage.GetPrimAtPath(path)
    UsdPhysics.MaterialAPI.Apply(p).CreateStaticFrictionAttr().Set(sf)
    UsdPhysics.MaterialAPI.Apply(p).CreateDynamicFrictionAttr().Set(df)
    PhysxSchema.PhysxMaterialAPI.Apply(p).CreateFrictionCombineModeAttr().Set(combine)
    return UsdShade.Material(p)

def make_box(path, lx, ly, lz, tx, ty, tz,
             rx=0.0, ry=0.0, rz=0.0, mat=None, dynamic=False):
    """
    创建Box
    lx/ly/lz : 实际尺寸(m)
    tx/ty/tz : 中心坐标(m)
    rx/ry/rz : 欧拉角(度) XYZ顺序
    """
    remove_if_exists(path)
    c = UsdGeom.Cube.Define(stage, path)
    c.GetSizeAttr().Set(1.0)
    p = c.GetPrim()
    # 用xformOp直接写，避免XformCommonAPI多层叠加问题
    ops = UsdGeom.Xformable(p).AddXformOp
    p.GetAttribute("xformOp:translate").Set(Gf.Vec3d(tx, ty, tz)) \
        if p.HasAttribute("xformOp:translate") else \
        UsdGeom.Xformable(p).AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
    # 用XformCommonAPI统一设置
    xf = UsdGeom.XformCommonAPI(p)
    xf.SetTranslate(Gf.Vec3d(tx, ty, tz))
    xf.SetScale(Gf.Vec3f(lx, ly, lz))
    if rx or ry or rz:
        xf.SetRotate(Gf.Vec3f(rx, ry, rz),
                     UsdGeom.XformCommonAPI.RotationOrderXYZ)
    UsdPhysics.CollisionAPI.Apply(p)
    if dynamic:
        UsdPhysics.RigidBodyAPI.Apply(p)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(p).Bind(mat)
    return p

# ════════════════════════════════════════════════════════
# Step 0: 重力
# ════════════════════════════════════════════════════════
scene = UsdPhysics.Scene.Get(stage, "/World/PhysicsScene")
scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
scene.GetGravityMagnitudeAttr().Set(9.81)
print("✓ 重力: (0,0,-1) 9.81m/s²")

# ════════════════════════════════════════════════════════
# Step 1: Base_Cube（底面对齐地面Z=0）
# ════════════════════════════════════════════════════════
# ★ Base_Cube的XY尺寸和XY位置可在此调整
BC_SX = 0.60   # ★ Base_Cube X尺寸
BC_SY = 0.60   # ★ Base_Cube Y尺寸
BC_TX = 0.00   # ★ Base_Cube X位置
BC_TY = 0.00   # ★ Base_Cube Y位置

remove_if_exists("/World/Base_Cube")
p = make_box("/World/Base_Cube",
    BC_SX, BC_SY, BASE_H,
    BC_TX, BC_TY, BASE_H/2)   # Z中心=BASE_H/2，底面Z=0，顶面Z=BASE_H
print(f"✓ Base_Cube: {BC_SX}×{BC_SY}×{BASE_H}m | 底面Z=0 顶面Z={BASE_H}m")

# ════════════════════════════════════════════════════════
# Step 2: GroundPlane碰撞确认
# ════════════════════════════════════════════════════════
for gp_path in ["/World/GroundPlane/CollisionPlane",
                "/World/GroundPlane/CollisionMesh"]:
    gp = stage.GetPrimAtPath(gp_path)
    if gp.IsValid():
        if not UsdPhysics.CollisionAPI.Get(stage, gp.GetPath()):
            UsdPhysics.CollisionAPI.Apply(gp)
        print(f"✓ {gp_path} 碰撞确认")

# ════════════════════════════════════════════════════════
# Step 3: equipment碰撞确认（位置由UI手动调整）
# ════════════════════════════════════════════════════════
mesh = stage.GetPrimAtPath("/World/equipment/node_/mesh_")
if mesh.IsValid():
    if not UsdPhysics.CollisionAPI.Get(stage, mesh.GetPath()):
        UsdPhysics.CollisionAPI.Apply(mesh)
    PhysxSchema.PhysxCollisionAPI.Apply(mesh)
    print("✓ equipment/mesh_ 碰撞确认")

# ════════════════════════════════════════════════════════
# Step 4: 摩擦材质
# ════════════════════════════════════════════════════════
if not stage.GetPrimAtPath("/World/Mat").IsValid():
    UsdGeom.Xform.Define(stage, "/World/Mat")

roller_mat = make_material("/World/Mat/Roller",
    MU_ROLLER_S, MU_ROLLER_D)   # 不锈钢滚轮等效
rail_mat   = make_material("/World/Mat/Rail",
    MU_RAIL_S, MU_RAIL_D)       # 不锈钢导轨
surf_mat   = make_material("/World/Mat/Surface",
    MU_SURF_S, MU_SURF_D)       # 后段接触面（等效滚轮）
pallet_mat = make_material("/World/Mat/Pallet",
    MU_PALLET_S, MU_PALLET_D)   # 塑料托盘
print("✓ 摩擦材质创建完成")

# ════════════════════════════════════════════════════════
# Step 5: 清空并重建Conveyor
# ════════════════════════════════════════════════════════
if not stage.GetPrimAtPath("/World/Conveyor").IsValid():
    UsdGeom.Xform.Define(stage, "/World/Conveyor")
else:
    for child in list(
        stage.GetPrimAtPath("/World/Conveyor").GetChildren()
    ):
        stage.RemovePrim(child.GetPath())
print("✓ Conveyor已清空")

# ════════════════════════════════════════════════════════
# Step 6: 前段18个滚轮（不锈钢，解析碰撞）
# ════════════════════════════════════════════════════════
for i in range(ROLLER_N):
    x = i * ROLLER_P
    z = ROLLER_Z0 - x * math.sin(AR)
    for side, ys in [("L", -1), ("R", +1)]:
        path = f"/World/Conveyor/Roller_{i:02d}_{side}"
        cyl  = UsdGeom.Cylinder.Define(stage, path)
        cyl.GetRadiusAttr().Set(ROLLER_R)
        cyl.GetHeightAttr().Set(ROLLER_L)
        cyl.GetAxisAttr().Set("Y")
        UsdGeom.XformCommonAPI(cyl.GetPrim()).SetTranslate(
            Gf.Vec3d(x, ys * ROLLER_Y, z))
        UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
        UsdShade.MaterialBindingAPI.Apply(
            cyl.GetPrim()).Bind(roller_mat)

print(f"✓ 前段滚轮: {ROLLER_N*2}个"
      f" | X=0~{(ROLLER_N-1)*ROLLER_P*1000:.0f}mm"
      f" | 轴心Y=±{ROLLER_Y*1000:.1f}mm")

# ════════════════════════════════════════════════════════
# Step 7: 前段导轨（内收5.3° + 坡度4°）
#   底面平行于输送面：绕Y轴旋转-4°（沿坡面方向）
#   内收：绕X轴旋转±5.3°
# ════════════════════════════════════════════════════════
fwd_xc  = FWD_L / 2
# 前段导轨中心Y（内侧面到中心）
fwd_y_in = INLET_W/2 - fwd_xc * math.tan(TILT_R)
fwd_yc   = fwd_y_in + RAIL_T/2
# 前段导轨中心Z（滚轮顶面+导轨半高）
fwd_zc   = z_top(fwd_xc) + RAIL_H/2

for side, ys, rz in [
    ("Left",  -1, +TILT_DEG),   # 左轨：绕Z正转内收
    ("Right", +1, -TILT_DEG),   # 右轨：绕Z负转内收
]:
    p = make_box(
        f"/World/Conveyor/Rail_{side}_forward",
        RFWD_L, RAIL_T, RAIL_H,             # 尺寸
        fwd_xc, ys*fwd_yc, fwd_zc,          # 中心坐标
        rx=0.0,                              # ★ X轴旋转（内倾微调）
        ry=-ANGLE_DEG,                       # 底面平行坡面
        rz=rz,                               # 内收角
        mat=rail_mat
    )

print(f"✓ 前段导轨: 实长{RFWD_L*1000:.1f}mm"
      f" | 进料口{INLET_W*1000:.1f}mm→{FWD_W*1000:.0f}mm"
      f" | 坡度{ANGLE_DEG}°+内收{TILT_DEG}°")

# ════════════════════════════════════════════════════════
# Step 8: 后段导轨（垂直，坡度4°）
#   底面平行于输送面：绕Y轴旋转-4°
# ════════════════════════════════════════════════════════
bwd_xc = FWD_L + BWD_L/2
bwd_yc = BWD_W/2 + RAIL_T/2
bwd_zc = z_top(bwd_xc) + RAIL_H/2

for side, ys in [("Left", -1), ("Right", +1)]:
    p = make_box(
        f"/World/Conveyor/Rail_{side}_backward",
        BWD_L, RAIL_T, RAIL_H,
        bwd_xc, ys*bwd_yc, bwd_zc,
        rx=0.0,           # ★ X轴旋转（微调）
        ry=-ANGLE_DEG,    # 底面平行坡面
        rz=0.0,           # 后段无内收
        mat=rail_mat
    )

print(f"✓ 后段导轨: {BWD_L*1000:.0f}mm"
      f" | 内侧间距{BWD_W*1000:.0f}mm"
      f" | 坡度{ANGLE_DEG}°")

# ════════════════════════════════════════════════════════
# Step 9: 后段接触面（取代22个小滚轮，坡度4°）
#   整体倾斜4°，顶面与输送坡面平行
#   摩擦等效不锈钢小滚轮
# ════════════════════════════════════════════════════════
bwd_surf_zc = z_top(bwd_xc) - SURF_T/2

p = make_box(
    "/World/Conveyor/Backward_Surface",
    BWD_L, BWD_W, SURF_T,
    bwd_xc, 0.0, bwd_surf_zc,
    rx=0.0,
    ry=-ANGLE_DEG,    # 顶面平行坡面
    rz=0.0,
    mat=surf_mat
)
print(f"✓ 后段接触面: {BWD_L*1000:.0f}×{BWD_W*1000:.0f}mm"
      f" | 取代22个小滚轮 | 坡度{ANGLE_DEG}°")

# ════════════════════════════════════════════════════════
# Step 10: 托盘CollisionProxy（动态刚体，塑料）
# ════════════════════════════════════════════════════════
if not stage.GetPrimAtPath("/World/Pallet").IsValid():
    UsdGeom.Xform.Define(stage, "/World/Pallet")
remove_if_exists("/World/Pallet/CollisionProxy")

# 初始位置：前段入口居中，滚轮顶面上方5mm（★ 可调）
PAL_X0  = PALLET_X / 2          # ★ 托盘初始X中心
PAL_Y0  = 0.000                  # ★ 托盘初始Y偏心（0=居中，正值向右偏）
PAL_GAP = 0.005                  # ★ 托盘与滚轮顶面间隙（防穿模）
PAL_Z0  = z_top(PAL_X0) + PALLET_H/2 + PAL_GAP

proxy = UsdGeom.Cube.Define(stage, "/World/Pallet/CollisionProxy")
proxy.GetSizeAttr().Set(1.0)
pp = proxy.GetPrim()
xf = UsdGeom.XformCommonAPI(pp)
xf.SetTranslate(Gf.Vec3d(PAL_X0, PAL_Y0, PAL_Z0))
xf.SetScale(Gf.Vec3f(PALLET_X, PALLET_Y, PALLET_H))
xf.SetRotate(Gf.Vec3f(-ANGLE_DEG, 0.0, 0.0),       # 平行输送面
             UsdGeom.XformCommonAPI.RotationOrderXYZ)

rb = UsdPhysics.RigidBodyAPI.Apply(pp)
rb.CreateVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
UsdPhysics.MassAPI.Apply(pp).CreateMassAttr().Set(PALLET_MASS)
UsdPhysics.CollisionAPI.Apply(pp)
UsdShade.MaterialBindingAPI.Apply(pp).Bind(pallet_mat)

print(f"✓ 托盘: {PALLET_X*1000:.0f}×{PALLET_Y*1000:.0f}×{PALLET_H*1000:.0f}mm"
      f" | 初始({PAL_X0:.3f}, {PAL_Y0:.3f}, {PAL_Z0:.3f})m"
      f" | 偏心Y={PAL_Y0*1000:.1f}mm")

# ════════════════════════════════════════════════════════
# Step 11: 物理验证摘要
# ════════════════════════════════════════════════════════
mu_eff  = MU_ROLLER_S * MU_PALLET_S   # multiply combine
tan_a   = math.tan(AR)
gap_bwd = (BWD_W - PALLET_X) / 2 * 1000  # 后段侧向间隙mm

print(f"""
══ 物理验证摘要 ══
坡度角              : {ANGLE_DEG}°  tan={tan_a:.4f}
等效μ(滚轮×托盘)   : {mu_eff:.4f}
下滑条件            : {'✓ μ < tanθ，托盘应能下滑' if mu_eff < tan_a else '⚠ μ > tanθ，托盘可能静止，降低MU_ROLLER_S'}

间隙分析：
  进料口侧向间隙    : ±{(INLET_W-PALLET_X)*500:.1f}mm/侧
  后段侧向间隙      : ±{gap_bwd:.1f}mm/侧  {'← ⚠ 极紧，卡住高危区' if gap_bwd < 2 else ''}

高度：
  第一列滚轮顶面Z   : {z_top(0):.3f}m
  后段末端Z         : {z_top(FWD_L+BWD_L):.3f}m
  总高度差          : {(z_top(0)-z_top(FWD_L+BWD_L))*1000:.1f}mm

★ 标定步骤：
  1. 按Play，托盘居中(Y=0)应缓慢下滑
  2. 若静止 → 减小MU_ROLLER_S（当前{MU_ROLLER_S}）
  3. 若顺利下滑 → 增大PAL_Y0直到卡住，记录临界值

✅ 构建完成，按Play测试
""")