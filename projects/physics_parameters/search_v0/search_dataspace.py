import math, json
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf
import omni.usd

stage = omni.usd.get_context().get_stage()

# ════════════════════════════════════════
# ★ 可调参数区
# ════════════════════════════════════════

# ── 摩擦系数 ────────────────────────────
# 等效μ = μ_roller_s × μ_pallet_s（multiply combine）
# 当前: 0.060 × 0.40 = 0.0240
MU_ROLLER_S_BASE = 0.060    # ★ Roller的μs基准值（UI里查到的）
PALLET_MU_S      = 0.400    # ★ Pallet的μs（固定不变）
MU_ROLLER_BASE   = MU_ROLLER_S_BASE * PALLET_MU_S  # = 0.0240 等效μ

# 域随机化：随机化Roller μs，使等效μ在±30%范围内
MU_RAND_RANGE    = 0.30     # ★ ±30%

# ── 搜索空间 ────────────────────────────
Y_SEARCH   = np.linspace(0.000, 0.040, 9)  # ★ Y偏心 0~40mm，9个点
YAW_SEARCH = np.linspace(0.0,   8.0,   9)  # ★ Yaw角 0~8°，9个点
Y_GRID     = np.linspace(0.000, 0.040, 7)  # ★ 网格Y轴 7点
YAW_GRID   = np.linspace(0.0,   8.0,   7)  # ★ 网格Yaw轴 7点

# ── 仿真参数 ─────────────────────────────
N_TRIALS     = 5      # ★ 每格重复次数
SIM_DURATION = 5.0    # ★ 单次仿真时长(秒)
STUCK_SPEED  = 0.005  # ★ 卡住判定速度阈值(m/s)

SAVE_PATH = "/tmp/safe_pose_set.json"  # ★ 结果保存路径

# ════════════════════════════════════════
# 场景参数（自动从场景读取，勿改）
# ════════════════════════════════════════
def get_scene_baseline():
    r00 = stage.GetPrimAtPath("/World/ConveyorGroup/Roller_00_L")
    cg  = stage.GetPrimAtPath("/World/ConveyorGroup")
    if not r00.IsValid():
        raise RuntimeError("Roller_00_L不存在，检查路径")
    r_z  = r00.GetAttribute("xformOp:translate").Get()[2]
    cg_z = cg.GetAttribute("xformOp:translate").Get()[2] if cg.IsValid() else 0.0
    return r_z + cg_z + 0.010  # 轴心Z + 半径

ROLLER_TOP_Z0  = get_scene_baseline()
AR             = math.radians(4.0)
FWD_L          = 0.228
BWD_L          = 0.296
# CONVEYOR_END_X = FWD_L + BWD_L        # 0.524m
# SUCCESS_X      = CONVEYOR_END_X * 0.9 # 成功判定X
CONVEYOR_END_X = 0.500     # ★ Stopper实际X=0.5m
SUCCESS_X      = CONVEYOR_END_X * 0.9  # = 0.450m

# 从场景读取托盘尺寸
_pp  = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
_ps  = _pp.GetAttribute("xformOp:scale").Get()
PALLET_X, PALLET_Y, PALLET_H = _ps[0], _ps[1], _ps[2]
PAL_X0 = PALLET_X / 4  # 固定X，3/4在滚轮上


# ── 从实际场景读取的基准位置（覆盖计算值）──
PAL_X0_ACTUAL = 0.045      # ★ 从UI读取的实际X
PAL_Z0_ACTUAL = 1.26751    # ★ 从UI读取的实际Z（托盘中心）
PAL_Y0_CENTER = 0.00253    # ★ 实际居中Y（近似为0）

def roller_top_z(x):
    return ROLLER_TOP_Z0 - x * math.sin(AR)

# ════════════════════════════════════════
# 工具函数
# ════════════════════════════════════════

def set_pallet_pose(y, yaw_deg, pitch_extra=0.0):
    """重置托盘位姿，以实际场景基准位置为准"""
    p = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
    if not p.IsValid():
        return False

    # ★ 用实际Z，不重新计算
    xf = UsdGeom.XformCommonAPI(p)
    xf.SetTranslate(Gf.Vec3d(
        PAL_X0_ACTUAL,
        PAL_Y0_CENTER + y,   # 在实际居中值基础上叠加偏心量
        PAL_Z0_ACTUAL
    ))
    xf.SetRotate(
        Gf.Vec3f(0.0, 4.0 + pitch_extra, yaw_deg),
        UsdGeom.XformCommonAPI.RotationOrderXYZ
    )

    rb = UsdPhysics.RigidBodyAPI.Get(stage, p.GetPath())
    if rb:
        rb.GetVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
        rb.GetAngularVelocityAttr().Set(Gf.Vec3f(0, 0, 0))
    return True

def randomize_mu():
    """
    域随机化Roller μs
    等效μ = μ_roller_s × PALLET_MU_S
    随机化μ_roller_s，使等效μ在 MU_ROLLER_BASE ± MU_RAND_RANGE 内
    """
    mat = stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return

    # 等效μ随机值
    mu_eff_new = MU_ROLLER_BASE * (
        1.0 + np.random.uniform(-MU_RAND_RANGE, MU_RAND_RANGE)
    )
    # 限制在合理范围（不能超过tan4°否则肯定卡住）
    mu_eff_new = float(np.clip(mu_eff_new, 0.005, 0.065))

    # 反推Roller μs
    mu_s = mu_eff_new / PALLET_MU_S
    mu_d = mu_s * 0.7

    m = UsdPhysics.MaterialAPI.Get(stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(mu_s)
        m.GetDynamicFrictionAttr().Set(mu_d)

def restore_mu():
    """搜索结束后恢复基准μ"""
    mat = stage.GetPrimAtPath("/World/Mat/Roller")
    if not mat.IsValid():
        return
    m = UsdPhysics.MaterialAPI.Get(stage, mat.GetPath())
    if m:
        m.GetStaticFrictionAttr().Set(MU_ROLLER_S_BASE)
        m.GetDynamicFrictionAttr().Set(MU_ROLLER_S_BASE * 0.7)

def get_pallet_state():
    p  = stage.GetPrimAtPath("/World/Pallet/CollisionProxy")
    if not p.IsValid():
        return None, None
    t  = p.GetAttribute("xformOp:translate").Get()
    rb = UsdPhysics.RigidBodyAPI.Get(stage, p.GetPath())
    v  = rb.GetVelocityAttr().Get() if rb else Gf.Vec3f(0, 0, 0)
    return t, v

def check_result(pos, vel):
    if pos is None:
        return 'error'
    speed = math.sqrt(vel[0]**2 + vel[1]**2 + vel[2]**2)

    if pos[0] >= SUCCESS_X:
        return 'success'                    # 到达末端90%处
    elif pos[2] < PAL_Z0_ACTUAL - 0.15:
        return 'fallen'                     # 掉落（低于初始Z 15cm）
    elif speed < STUCK_SPEED:
        return 'stuck'                      # 速度接近0
    else:
        return 'timeout'                    # 超时未到达

def run_one_trial(y, yaw, pitch_extra=0.0):
    """
    单次仿真（阻塞步进）
    运行前需先按Play启动仿真
    """
    from omni.isaac.core import SimulationContext
    sim = SimulationContext.instance()
    if sim is None:
        sim = SimulationContext(
            physics_dt=1/60.0,
            rendering_dt=1/60.0
        )

    # 随机化摩擦 + 重置位姿
    randomize_mu()
    set_pallet_pose(y, yaw, pitch_extra)

    # 步进
    n_steps = int(SIM_DURATION * 60)
    for _ in range(n_steps):
        sim.step(render=False)

    pos, vel = get_pallet_state()
    return check_result(pos, vel)

def run_trials(y, yaw, pitch_extra=0.0, n=N_TRIALS):
    results = []
    for t in range(n):
        r = run_one_trial(y, yaw, pitch_extra)
        results.append(r)
        print(f"      试验{t+1}/{n}: {r}")
    rate = results.count('success') / n
    return rate, results

# ════════════════════════════════════════
# 记录系统
# ════════════════════════════════════════
LOG = {
    "config": {
        "MU_ROLLER_S_BASE": MU_ROLLER_S_BASE,
        "PALLET_MU_S"     : PALLET_MU_S,
        "MU_ROLLER_BASE"  : MU_ROLLER_BASE,
        "MU_RAND_RANGE"   : MU_RAND_RANGE,
        "N_TRIALS"        : N_TRIALS,
        "SIM_DURATION"    : SIM_DURATION,
        "PALLET_XYH"      : [PALLET_X, PALLET_Y, PALLET_H],
        "PAL_X0"          : PAL_X0,
        "ROLLER_TOP_Z0"   : ROLLER_TOP_Z0,
        "CONVEYOR_END_X"  : CONVEYOR_END_X,
    },
    "step2"       : [],
    "step3"       : [],
    "step4"       : {"Y_grid": [], "Yaw_grid": [], "matrix": []},
    "safe_poses"  : [],
    "safe_pose_set": {}
}

def save_log():
    with open(SAVE_PATH, "w") as f:
        json.dump(LOG, f, indent=2)
    print(f"      → 已保存: {SAVE_PATH}")

# ════════════════════════════════════════
# 主搜索流程
# ════════════════════════════════════════
print("=" * 60)
print("参数空间搜索")
print("=" * 60)
print(f"  μ_roller_s基准  : {MU_ROLLER_S_BASE}")
print(f"  μ_pallet_s      : {PALLET_MU_S}")
print(f"  等效μ基准       : {MU_ROLLER_BASE:.4f}")
print(f"  域随机化范围    : ±{MU_RAND_RANGE*100:.0f}%"
      f" → 等效μ [{MU_ROLLER_BASE*(1-MU_RAND_RANGE):.4f},"
      f" {MU_ROLLER_BASE*(1+MU_RAND_RANGE):.4f}]")
print(f"  tan(4°)         : {math.tan(AR):.4f}")
print(f"  托盘 X={PALLET_X*1000:.1f}"
      f" Y={PALLET_Y*1000:.1f} H={PALLET_H*1000:.1f}mm")
print(f"  滚轮顶面Z(X=0)  : {ROLLER_TOP_Z0:.4f}m")
print(f"  输送带末端X     : {CONVEYOR_END_X:.3f}m")
print(f"  成功判定X       : {SUCCESS_X:.3f}m")
print("=" * 60)

# ── Step 1: 基准验证 ──────────────────

print("\n[Step 1] 基准验证 (Y=0, Yaw=0, Pitch=4°精确)")
result = run_one_trial(y=0.0, yaw=0.0, randomize=False)
print(f"  结果: {result}")

if result != 'success':
    print(f"  ⚠ 基准未通过")
    print(f"  可能原因1: PAL_Z0_RELEASE太高，托盘弹跳")
    print(f"  可能原因2: MU_ROLLER_S_BASE太大")
    print(f"  建议: 减小PAL_Z0_RELEASE到+0.015m")
    save_log()
    raise SystemExit("基准未通过")

print(f"  ✓ 基准通过，开始搜索")

# ── Step 2: Y偏心搜索 ────────────────
print(f"\n[Step 2] Y偏心搜索")
print(f"  Yaw=0固定 | Y: {Y_SEARCH[0]*1000:.0f}"
      f"~{Y_SEARCH[-1]*1000:.0f}mm | {N_TRIALS}次/点\n")

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
    save_log()

    if rate < 0.8 and Y_CRITICAL is None:
        Y_CRITICAL = y
        print(f"  ★ Y_critical ≈ {y*1000:.1f}mm")

if Y_CRITICAL is None:
    Y_CRITICAL = float(Y_SEARCH[-1])
    print(f"  ★ 全程通过，Y_critical = {Y_CRITICAL*1000:.1f}mm（上限）")

print(f"\nStep2完成 | Y_critical = {Y_CRITICAL*1000:.1f}mm")

# ── Step 3: Yaw角搜索 ────────────────
Y_HALF = Y_CRITICAL * 0.5
print(f"\n[Step 3] Yaw角搜索")
print(f"  Y={Y_HALF*1000:.1f}mm固定 | Yaw: {YAW_SEARCH[0]:.0f}"
      f"~{YAW_SEARCH[-1]:.0f}° | {N_TRIALS}次/点\n")

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
        print(f"  ★ Yaw_critical ≈ {yaw:.1f}°")

if YAW_CRITICAL is None:
    YAW_CRITICAL = float(YAW_SEARCH[-1])

print(f"\nStep3完成 | Yaw_critical = {YAW_CRITICAL:.1f}°")

# ── Step 4: Y×Yaw 网格 ───────────────
print(f"\n[Step 4] Y×Yaw 网格搜索")
print(f"  {len(Y_GRID)}×{len(YAW_GRID)} = {len(Y_GRID)*len(YAW_GRID)}格"
      f" × {N_TRIALS}次 = {len(Y_GRID)*len(YAW_GRID)*N_TRIALS}次仿真")
print(f"  预计时间: ~{len(Y_GRID)*len(YAW_GRID)*N_TRIALS*SIM_DURATION/60:.0f}分钟\n")

grid = np.zeros((len(Y_GRID), len(YAW_GRID)))
LOG["step4"]["Y_grid"]   = [round(float(y)*1000, 1) for y in Y_GRID]
LOG["step4"]["Yaw_grid"] = [round(float(w), 1) for w in YAW_GRID]

for i, y in enumerate(Y_GRID):
    row = []
    for j, yaw in enumerate(YAW_GRID):
        print(f"  [{i+1}/{len(Y_GRID)}, {j+1}/{len(YAW_GRID)}]"
              f" Y={y*1000:.1f}mm Yaw={yaw:.1f}°:")
        rate, raw = run_trials(float(y), float(yaw))
        grid[i, j] = rate
        row.append(round(rate, 3))

        if rate >= 0.80:
            LOG["safe_poses"].append({
                "Y_mm"   : round(float(y)*1000, 1),
                "Yaw_deg": round(float(yaw), 1),
                "rate"   : round(rate, 3)
            })

    LOG["step4"]["matrix"].append(row)
    save_log()

    row_str = " ".join(
        "✓" if v>=0.8 else ("△" if v>=0.4 else "✗")
        for v in grid[i]
    )
    print(f"\n  Y={y*1000:5.1f}mm │ {row_str}\n")

# ── Step 5: Safe Pose Set ─────────────
print("=" * 60)
print("[Step 5] Safe Pose Set 输出")
print("=" * 60)

safe = LOG["safe_poses"]
restore_mu()  # 恢复基准μ

if safe:
    y_vals   = [p["Y_mm"]    for p in safe]
    yaw_vals = [p["Yaw_deg"] for p in safe]

    LOG["safe_pose_set"] = {
        "position": {
            "X_m"       : PAL_X0,
            "Y_range_mm": [min(y_vals), max(y_vals)],
            "Z_m"       : roller_top_z(PAL_X0),
        },
        "orientation": {
            "Roll_deg"     : 0.0,
            "Pitch_deg"    : 4.0,
            "Yaw_range_deg": [min(yaw_vals), max(yaw_vals)],
        },
        "friction": {
            "mu_roller_s_base": MU_ROLLER_S_BASE,
            "mu_pallet_s"     : PALLET_MU_S,
            "mu_eff_base"     : MU_ROLLER_BASE,
            "mu_eff_range"    : [
                round(MU_ROLLER_BASE*(1-MU_RAND_RANGE), 4),
                round(MU_ROLLER_BASE*(1+MU_RAND_RANGE), 4)
            ],
        },
        "critical": {
            "Y_critical_mm"   : round(Y_CRITICAL*1000, 1),
            "Yaw_critical_deg": round(YAW_CRITICAL, 1),
        },
        "stats": {
            "n_safe"    : len(safe),
            "total_grid": len(Y_GRID) * len(YAW_GRID),
            "safe_ratio": round(len(safe)/(len(Y_GRID)*len(YAW_GRID)), 3),
        }
    }
    save_log()

    print(f"""
  Y安全范围    : {min(y_vals):.1f} ~ {max(y_vals):.1f} mm
  Yaw安全范围  : {min(yaw_vals):.1f} ~ {max(yaw_vals):.1f} °
  Y_critical   : {Y_CRITICAL*1000:.1f} mm
  Yaw_critical : {YAW_CRITICAL:.1f} °
  安全格数     : {len(safe)} / {len(Y_GRID)*len(YAW_GRID)}
                 ({len(safe)/(len(Y_GRID)*len(YAW_GRID))*100:.0f}%)

  → Safe Pose Set 已保存: {SAVE_PATH}
  → 此区间作为RL训练目标位姿
""")
else:
    print("  ⚠ 未找到安全区域")
    print(f"  建议: 降低 MU_ROLLER_S_BASE"
          f"（当前{MU_ROLLER_S_BASE}）或缩小Y_GRID/YAW_GRID范围")
    save_log()

print("✅ 搜索完成")