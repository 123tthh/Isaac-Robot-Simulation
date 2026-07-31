import math, json, time
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd
import omni.timeline
import omni.kit.app

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# ★ 可调参数区
# ════════════════════════════════════════

# ── 从场景实际读取基准 ──────────────────
def get_scene_baseline():
    """读取滚轮实际世界Z，不依赖硬编码"""
    r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
    cg  = stage.GetPrimAtPath("/World/ConveyorGroup")
    if not r00.IsValid():
        raise RuntimeError("Roller_00_L不存在")
    r_z  = r00.GetAttribute("xformOp:translate").Get()[2]
    cg_z = cg.GetAttribute("xformOp:translate").Get()[2] if cg.IsValid() else 0.0
    return r_z + cg_z + 0.010   # 轴心Z + 半径

ROLLER_TOP_Z0  = get_scene_baseline()
AR             = math.radians(4.0)
FWD_L          = 0.228
BWD_L          = 0.296
CONVEYOR_END_X = FWD_L + BWD_L   # 0.524m

# 托盘实际尺寸（从场景读取）
proxy_prim = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
_ps = proxy_prim.GetAttribute("xformOp:scale").Get()
PALLET_X, PALLET_Y, PALLET_H = _ps[0], _ps[1], _ps[2]
PAL_X0 = PALLET_X / 4   # 固定X，3/4在滚轮上

def roller_top_z(x):
    return ROLLER_TOP_Z0 - x * math.sin(AR)

# ── ★ 标定后填入 ────────────────────────
MU_ROLLER_BASE = 0.0240   # ★ 等效μ（Roller×Pallet multiply结果）

# ── ★ 搜索空间 ──────────────────────────
Y_SEARCH  = np.linspace(0.000, 0.040, 9)   # ★ 0~40mm
YAW_SEARCH= np.linspace(0.0,   8.0,   9)   # ★ 0~8°
Y_GRID    = np.linspace(0.000, 0.040, 7)   # ★ 7×7网格
YAW_GRID  = np.linspace(0.0,   8.0,   7)

# ── ★ 仿真参数 ──────────────────────────
N_TRIALS      = 5          # ★ 每格重复次数
# 域随机化范围（±30%覆盖不锈钢-塑料摩擦不确定性）
MU_RAND_RANGE  = 0.30     # 0.0240 × (0.7~1.3) = 0.0168~0.0312
SIM_DURATION  = 5.0        # ★ 每次仿真时长(秒)
STUCK_SPEED   = 0.005      # ★ 判定卡住的速度阈值(m/s)
SUCCESS_X     = CONVEYOR_END_X * 0.9  # 成功判定X

SAVE_PATH = "/tmp/safe_pose_set.json"  # ★ 结果保存路径

# ════════════════════════════════════════
# 工具函数
# ════════════════════════════════════════

def set_pallet_pose(y, yaw_deg, pitch_extra=0.0):
    """设置托盘初始位姿，重置速度"""
    p = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
    if not p.IsValid():
        return False
    pal_z = roller_top_z(PAL_X0) + PALLET_H / 2
    xf = UsdGeom.XformCommonAPI(p)
    xf.SetTranslate(Gf.Vec3d(PAL_X0, y, pal_z))
    xf.SetRotate(
        Gf.Vec3f(0.0, 4.0 + pitch_extra, yaw_deg),
        UsdGeom.XformCommonAPI.RotationOrderXYZ
    )
    rb = UsdPhysics.RigidBodyAPI.Get(stage, p.GetPath())
    if rb:
        rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
    return True

def set_mu(mu_static):
    """更新滚轮摩擦系数"""
    mat = stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return
    m = UsdPhysics.MaterialAPI.Get(stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(float(mu_static))
        m.GetDynamicFrictionAttr().Set(float(mu_static * 0.7))

def get_pallet_state():
    """读取托盘位置和速度"""
    p  = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
    if not p.IsValid():
        return None, None
    t  = p.GetAttribute("xformOp:translate").Get()
    rb = UsdPhysics.RigidBodyAPI.Get(stage, p.GetPath())
    v  = rb.GetVelocityAttr().Get() if rb else Gf.Vec3f(0, 0, 0)
    return t, v

def check_result(pos, vel):
    """判定仿真结果"""
    if pos is None:
        return 'error'
    speed = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)
    if pos[0] >= SUCCESS_X:
        return 'success'
    elif pos[2] < ROLLER_TOP_Z0 - 0.15:
        return 'fallen'
    elif speed < STUCK_SPEED:
        return 'stuck'
    else:
        return 'timeout'

def run_one_trial_sync(y, yaw, pitch_extra=0.0):
    """
    同步运行单次仿真（阻塞）
    使用SimulationContext步进
    """
    from omni.isaac.core import SimulationContext
    sim = SimulationContext.instance()
    if sim is None:
        sim = SimulationContext(physics_dt=1/60.0, rendering_dt=1/60.0)

    # 设置位姿
    mu = MU_ROLLER_BASE * (1 + np.random.uniform(
        -MU_RAND_RANGE, MU_RAND_RANGE))
    set_mu(max(0.01, mu))
    set_pallet_pose(y, yaw, pitch_extra)

    # 步进仿真
    n_steps = int(SIM_DURATION * 60)
    for _ in range(n_steps):
        sim.step(render=False)   # render=False加速

    pos, vel = get_pallet_state()
    return check_result(pos, vel)

def run_trials(y, yaw, pitch_extra=0.0, n=N_TRIALS):
    """运行N次，返回(成功率, 结果列表)"""
    results = []
    for t in range(n):
        r = run_one_trial_sync(y, yaw, pitch_extra)
        results.append(r)
        print(f"    试验{t+1}/{n}: {r}")
    rate = results.count('success') / n
    return rate, results

# ════════════════════════════════════════
# 记录系统
# ════════════════════════════════════════
LOG = {
    "config": {
        "MU_ROLLER_BASE": MU_ROLLER_BASE,
        "MU_RAND_RANGE" : MU_RAND_RANGE,
        "N_TRIALS"      : N_TRIALS,
        "SIM_DURATION"  : SIM_DURATION,
        "PALLET_XYH"    : [PALLET_X, PALLET_Y, PALLET_H],
        "PAL_X0"        : PAL_X0,
        "ROLLER_TOP_Z0" : ROLLER_TOP_Z0,
    },
    "step2": [],
    "step3": [],
    "step4": {"Y_grid": [], "Yaw_grid": [], "matrix": []},
    "safe_poses": [],
    "safe_pose_set": {}
}

def save_log():
    with open(SAVE_PATH, "w") as f:
        json.dump(LOG, f, indent=2)
    print(f"  → 已保存: {SAVE_PATH}")

# ════════════════════════════════════════
# 主搜索流程
# ════════════════════════════════════════
print("=" * 60)
print("参数空间搜索开始")
print(f"  MU_ROLLER_BASE = {MU_ROLLER_BASE}")
print(f"  托盘尺寸: X={PALLET_X*1000:.1f} Y={PALLET_Y*1000:.1f} H={PALLET_H*1000:.1f}mm")
print(f"  滚轮顶面Z(X=0): {ROLLER_TOP_Z0:.4f}m")
print("=" * 60)

# ── Step 1: 基准确认 ──
print("""
[Step 1] 手动基准标定
  请先在场景里按Play，观察托盘居中(Y=0,Yaw=0)能否下滑
  调整顶部 MU_ROLLER_BASE 直到托盘匀加速
  完成后停止Play，重新运行此脚本继续Step2
  当前μ₀ = {:.3f}
""".format(MU_ROLLER_BASE))

# 确认基准（自动测一次居中）
print("[Step 1 自动验证] Y=0, Yaw=0 单次测试...")
baseline_result = run_one_trial_sync(y=0.0, yaw=0.0)
print(f"  基准结果: {baseline_result}")
if baseline_result != 'success':
    print(f"  ⚠ 基准未通过！请降低 MU_ROLLER_BASE（当前{MU_ROLLER_BASE}）后重跑")
    print(f"  建议值: MU_ROLLER_BASE = {MU_ROLLER_BASE * 0.8:.4f}")
    save_log()
    raise SystemExit("基准标定未通过，搜索终止")
else:
    print(f"  ✓ 基准通过，μ₀ = {MU_ROLLER_BASE}")

# ── Step 2: Y偏心搜索 ──
print(f"\n[Step 2] Y偏心搜索")
print(f"  范围: {Y_SEARCH[0]*1000:.0f}~{Y_SEARCH[-1]*1000:.0f}mm | {N_TRIALS}次/点\n")

Y_CRITICAL = None

for y in Y_SEARCH:
    print(f"  Y = {y*1000:.1f}mm:")
    rate, raw = run_trials(y, yaw=0.0)
    status = "✓下滑" if rate>=0.8 else ("△不稳" if rate>=0.4 else "✗卡住")
    print(f"  → 成功率={rate:.0%} {status}")

    LOG["step2"].append({
        "Y_mm": round(y*1000, 1),
        "rate": round(rate, 3),
        "raw" : raw
    })
    save_log()   # 每步保存

    if rate < 0.8 and Y_CRITICAL is None:
        Y_CRITICAL = y
        print(f"  ★ Y_critical = {y*1000:.1f}mm")

if Y_CRITICAL is None:
    Y_CRITICAL = Y_SEARCH[-1]
    print(f"  ★ 未找到临界，使用最大值{Y_CRITICAL*1000:.1f}mm")

print(f"\nStep2完成 | Y_critical = {Y_CRITICAL*1000:.1f}mm")

# ── Step 3: Yaw角搜索 ──
Y_HALF = Y_CRITICAL * 0.5
print(f"\n[Step 3] Yaw角搜索 (Y={Y_HALF*1000:.1f}mm)")
print(f"  范围: {YAW_SEARCH[0]:.0f}~{YAW_SEARCH[-1]:.0f}° | {N_TRIALS}次/点\n")

YAW_CRITICAL = None

for yaw in YAW_SEARCH:
    print(f"  Yaw = {yaw:.1f}°:")
    rate, raw = run_trials(Y_HALF, yaw=yaw)
    status = "✓" if rate>=0.8 else ("△" if rate>=0.4 else "✗")
    print(f"  → 成功率={rate:.0%} {status}")

    LOG["step3"].append({
        "Yaw_deg": round(yaw, 1),
        "rate"   : round(rate, 3),
        "raw"    : raw
    })
    save_log()

    if rate < 0.8 and YAW_CRITICAL is None:
        YAW_CRITICAL = yaw
        print(f"  ★ Yaw_critical = {yaw:.1f}°")

if YAW_CRITICAL is None:
    YAW_CRITICAL = YAW_SEARCH[-1]

print(f"\nStep3完成 | Yaw_critical = {YAW_CRITICAL:.1f}°")

# ── Step 4: Y×Yaw 二维网格 ──
print(f"\n[Step 4] Y×Yaw网格搜索")
print(f"  {len(Y_GRID)}×{len(YAW_GRID)}网格 | {N_TRIALS}次/格")
print(f"  总计: {len(Y_GRID)*len(YAW_GRID)*N_TRIALS}次仿真\n")

grid = np.zeros((len(Y_GRID), len(YAW_GRID)))
LOG["step4"]["Y_grid"]   = [round(y*1000,1) for y in Y_GRID]
LOG["step4"]["Yaw_grid"] = [round(yaw,1) for yaw in YAW_GRID]

for i, y in enumerate(Y_GRID):
    row = []
    for j, yaw in enumerate(YAW_GRID):
        print(f"  [{i},{j}] Y={y*1000:.1f}mm Yaw={yaw:.1f}°:")
        rate, raw = run_trials(y, yaw)
        grid[i, j] = rate
        row.append(round(rate, 3))

        if rate >= 0.80:
            LOG["safe_poses"].append({
                "Y_mm"   : round(y*1000, 1),
                "Yaw_deg": round(yaw, 1),
                "rate"   : round(rate, 3)
            })

    LOG["step4"]["matrix"].append(row)
    save_log()

    # 打印当前行热力图
    row_str = " ".join(
        "✓" if v>=0.8 else ("△" if v>=0.4 else "✗")
        for v in grid[i]
    )
    print(f"\n  Y={y*1000:5.1f}mm │ {row_str}\n")

# ── Step 5: 输出Safe Pose Set ──
print("=" * 60)
print("[Step 5] Safe Pose Set")
print("=" * 60)

safe = LOG["safe_poses"]
if safe:
    y_vals   = [p["Y_mm"]    for p in safe]
    yaw_vals = [p["Yaw_deg"] for p in safe]

    LOG["safe_pose_set"] = {
        "position": {
            "X_m"    : PAL_X0,
            "Y_range_mm": [min(y_vals), max(y_vals)],
            "Z_m"    : roller_top_z(PAL_X0),
        },
        "orientation": {
            "Roll_deg"  : 0.0,
            "Pitch_deg" : 4.0,
            "Yaw_range_deg": [min(yaw_vals), max(yaw_vals)],
        },
        "mu_base"      : MU_ROLLER_BASE,
        "mu_range"     : [round(MU_ROLLER_BASE*(1-MU_RAND_RANGE),4),
                          round(MU_ROLLER_BASE*(1+MU_RAND_RANGE),4)],
        "Y_critical_mm": round(Y_CRITICAL*1000, 1),
        "Yaw_critical_deg": round(YAW_CRITICAL, 1),
        "n_safe"       : len(safe),
        "total_grid"   : len(Y_GRID) * len(YAW_GRID),
    }
    save_log()

    print(f"""
  Y安全范围   : {min(y_vals):.1f} ~ {max(y_vals):.1f} mm
  Yaw安全范围 : {min(yaw_vals):.1f} ~ {max(yaw_vals):.1f} °
  安全格数    : {len(safe)} / {len(Y_GRID)*len(YAW_GRID)}
  Y_critical  : {Y_CRITICAL*1000:.1f} mm
  Yaw_critical: {YAW_CRITICAL:.1f} °

  → RL训练目标位姿区间已确定
  → 结果已保存: {SAVE_PATH}
""")
else:
    print("  ⚠ 未找到安全区域，降低MU_ROLLER_BASE后重跑")
    save_log()

print("✅ 搜索完成")