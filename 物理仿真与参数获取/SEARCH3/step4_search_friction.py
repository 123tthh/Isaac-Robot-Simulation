"""
step4_search_friction.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 4: 读 Step1/2/3 的 Y/Yaw/Pitch 安全区间
        ★ 只对 μ_roller_s 进行扫描（Y/Yaw/Pitch 不随机化）

策略：
  • Y, Yaw, Pitch 都固定为各自安全区间的中点
  • 只 μ_roller_s 在变（0.030 ~ 0.090）
  • 找出 μ 的可行区间，这是真正意义上的摩擦 DR 容差

最终产物 safe_pose.json 汇总：
  - Y / Yaw / Pitch 三个几何维度的安全区间（前三步的结果）
  - μ 的安全区间（本步结果）
  - 这些区间作为 RL 训练的 DR 采样域

输出：~/Desktop/user_scripts/result/safe_pose.json
═══════════════════════════════════════════════════════════════════
"""

import os
import sys
import json
import asyncio
import numpy as np

COMMON_DIR = os.path.expanduser("~/Desktop/user_scripts")
if COMMON_DIR not in sys.path:
    sys.path.insert(0, COMMON_DIR)

from safe_pose_common import (
    SceneHandle, run_one_trial, extract_safe_range,
    RESULT_DIR, MU_ROLLER_S_BASE, PALLET_MU_S,
    PAL_X0_ACTUAL, PAL_Y0_CENTER, PAL_Z0_ACTUAL,
)

# ═══════════════════════════════════════════════════════════════
# Step 4 参数
# ═══════════════════════════════════════════════════════════════

MU_SEARCH_S = np.arange(0.030, 0.095 + 0.001, 0.010)   # 0.03~0.09 共 7 点
N_TRIALS    = 5
Z_OFFSET    = 0.003

Y_INPUT     = os.path.join(RESULT_DIR, "safe_pose_y.json")
YAW_INPUT   = os.path.join(RESULT_DIR, "safe_pose_yaw.json")
PITCH_INPUT = os.path.join(RESULT_DIR, "safe_pose_pitch.json")
SAVE_PATH   = os.path.join(RESULT_DIR, "safe_pose.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 4] μ_roller_s 扫描 · 汇总最终 Safe Pose Set")
    print("  ★ 只扫描 μ，Y/Yaw/Pitch 固定为各自区间中点")
    print("═" * 62)

    # ── 读取前三步结果 ──────────────────────────────────────
    for p in [Y_INPUT, YAW_INPUT, PITCH_INPUT]:
        if not os.path.exists(p):
            print(f"✗ 缺少 {p}，请依次运行 step1/2/3")
            return

    with open(Y_INPUT) as f:
        y_log = json.load(f)
    with open(YAW_INPUT) as f:
        yaw_log = json.load(f)
    with open(PITCH_INPUT) as f:
        pitch_log = json.load(f)

    y_range     = y_log.get("safe_range_mm")       or [0.0, 0.0]
    yaw_range   = yaw_log.get("safe_range_deg")    or [0.0, 0.0]
    pitch_range = pitch_log.get("safe_range_deg")  or [4.0, 4.0]

    # 固定为各区间中点
    Y_FIXED_MM   = (y_range[0]     + y_range[1])     / 2.0
    Y_FIXED_M    = Y_FIXED_MM / 1000.0
    YAW_FIXED    = (yaw_range[0]   + yaw_range[1])   / 2.0
    PITCH_FIXED  = (pitch_range[0] + pitch_range[1]) / 2.0

    print(f"  Y 区间 → 固定  : [{y_range[0]:.1f},{y_range[1]:.1f}]mm → {Y_FIXED_MM:.1f}mm")
    print(f"  Yaw 区间 → 固定: [{yaw_range[0]:.1f},{yaw_range[1]:.1f}]° → {YAW_FIXED:.1f}°")
    print(f"  Pitch 区间 → 固: [{pitch_range[0]:.1f},{pitch_range[1]:.1f}]° → {PITCH_FIXED:.1f}°")
    print(f"  μ 搜索点       : {[round(float(m),3) for m in MU_SEARCH_S]}")
    print(f"  每点重复       : {N_TRIALS} 次")
    print(f"  输出           : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"                  : "step4_Friction_Final",
            "note"                  : "只扫描 μ，Y/Yaw/Pitch 固定",
            "Y_input_range_mm"      : y_range,
            "Yaw_input_range_deg"   : yaw_range,
            "Pitch_input_range_deg" : pitch_range,
            "Y_fixed_mm"            : round(Y_FIXED_MM, 1),
            "Yaw_fixed_deg"         : round(YAW_FIXED, 1),
            "Pitch_fixed_deg"       : round(PITCH_FIXED, 1),
            "MU_SEARCH_S"           : [round(float(m),4) for m in MU_SEARCH_S],
            "N_TRIALS"              : N_TRIALS,
            "Z_OFFSET_m"            : Z_OFFSET,
        },
        "trials"         : [],
        "summary"        : [],
        "safe_mu_s_range": None,
        "final_safe_pose": {},
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for mu_s in MU_SEARCH_S:
        print(f"\n  ── μ_roller_s = {mu_s:.3f} "
              f"(μ_eff ≈ {mu_s*PALLET_MU_S:.4f}) ──")
        successes = 0
        for t in range(N_TRIALS):
            rec = await run_one_trial(
                sh,
                y           = Y_FIXED_M,
                yaw_deg     = float(YAW_FIXED),
                pitch_deg   = float(PITCH_FIXED),
                z_offset    = Z_OFFSET,
                mu_roller_s = float(mu_s),
                tag         = f"μ={mu_s:.3f} #{t+1}/{N_TRIALS}",
            )
            rec["mu_roller_s"] = round(float(mu_s), 4)
            rec["mu_eff"]      = round(float(mu_s) * PALLET_MU_S, 4)
            rec["Y_mm"]        = round(Y_FIXED_MM, 1)
            rec["Yaw_deg"]     = round(float(YAW_FIXED), 1)
            rec["Pitch_deg"]   = round(float(PITCH_FIXED), 1)
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
            save()

        rate = successes / N_TRIALS
        rates.append(rate)
        status = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  → 成功率 = {rate:.0%}  {status}")

        log["summary"].append({
            "mu_roller_s": round(float(mu_s), 4),
            "mu_eff"     : round(float(mu_s) * PALLET_MU_S, 4),
            "rate"       : round(rate, 3),
        })
        save()

    mu_values = [float(m) for m in MU_SEARCH_S]
    safe_mu   = extract_safe_range(mu_values, rates, threshold=0.8)
    log["safe_mu_s_range"] = safe_mu

    # ── 汇总最终 Safe Pose Set ──────────────────────────────
    log["final_safe_pose"] = {
        "position": {
            "X_m"       : PAL_X0_ACTUAL,
            "Y_range_mm": y_range,
            "Y_center_m": PAL_Y0_CENTER,
            "Z_m"       : PAL_Z0_ACTUAL + Z_OFFSET,
            "Z_offset_m": Z_OFFSET,
        },
        "orientation": {
            "Roll_deg"      : 0.0,
            "Pitch_range_deg": pitch_range,
            "Yaw_range_deg"  : yaw_range,
        },
        "friction": {
            "mu_roller_s_range"  : safe_mu,
            "mu_roller_s_nominal": MU_ROLLER_S_BASE,
            "mu_pallet_s"        : PALLET_MU_S,
        },
    }
    save()

    print("\n" + "═" * 62)
    print("  ★ 最终 Safe Pose Set (RL 训练 DR 采样域)")
    print("═" * 62)
    print(f"    Y        : [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"    Yaw      : [{yaw_range[0]:.1f}, {yaw_range[1]:.1f}]°")
    print(f"    Pitch    : [{pitch_range[0]:.1f}, {pitch_range[1]:.1f}]°")
    print(f"    μ_roller : {safe_mu or '(未收敛)'}")
    print(f"\n  ✓ 写入: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
