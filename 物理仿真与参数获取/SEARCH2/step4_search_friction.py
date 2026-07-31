"""
step4_search_friction.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 4: 读 Step1/2/3 的安全区间，在这些区间内对 μ 进行扫描搜索，
        进一步收紧参数空间，产出最终 safe_pose.json

工作方式：
  • Y/Yaw/Pitch 都从各自安全区间随机采样
  • μ_roller_s 从一个扫描列表（不再只是 ±30% 随机）中逐点测试
  • 每个 μ 跑 N_TRIALS 次，统计成功率
  • 最后汇总所有阶段的安全区间，写入 safe_pose.json

输出：~/Desktop/user_scripts/result/safe_pose.json  ← 最终产物
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
    RESULT_DIR, MU_ROLLER_S_BASE, MU_ROLLER_BASE,
    PITCH_BASE, PALLET_MU_S,
    PAL_X0_ACTUAL, PAL_Y0_CENTER, PAL_Z0_ACTUAL,
)

# ═══════════════════════════════════════════════════════════════
# Step 4 专属参数
# ═══════════════════════════════════════════════════════════════

MU_SEARCH_S = np.arange(0.030, 0.095 + 0.001, 0.010)  # 0.03~0.09 共 7 点
N_TRIALS    = 8                                       # 每个 μ 重复次数（高一些）
Z_OFFSET    = 0.003                                    # 与 Step3 保持一致

Y_INPUT     = os.path.join(RESULT_DIR, "safe_pose_y.json")
YAW_INPUT   = os.path.join(RESULT_DIR, "safe_pose_yaw.json")
PITCH_INPUT = os.path.join(RESULT_DIR, "safe_pose_pitch.json")
SAVE_PATH   = os.path.join(RESULT_DIR, "safe_pose.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 4] μ 搜索 · 汇总最终 Safe Pose Set")
    print("═" * 62)

    # ── 读取前三步结果 ──────────────────────────────────────
    for p in [Y_INPUT, YAW_INPUT, PITCH_INPUT]:
        if not os.path.exists(p):
            print(f"✗ 缺少 {p}，请先跑完 step1/2/3")
            return

    with open(Y_INPUT) as f:
        y_log = json.load(f)
    with open(YAW_INPUT) as f:
        yaw_log = json.load(f)
    with open(PITCH_INPUT) as f:
        pitch_log = json.load(f)

    y_range     = y_log.get("safe_range_mm")       or [0.0, 40.0]
    yaw_range   = yaw_log.get("safe_range_deg")    or [0.0, 8.0]
    pitch_range = pitch_log.get("safe_range_deg")  or [4.0, 4.0]

    print(f"  Y 区间    : [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"  Yaw 区间  : [{yaw_range[0]:.1f}, {yaw_range[1]:.1f}]°")
    print(f"  Pitch 区间: [{pitch_range[0]:.1f}, {pitch_range[1]:.1f}]°")
    print(f"  μ 搜索    : {[round(m,3) for m in MU_SEARCH_S]}")
    print(f"  每点重复  : {N_TRIALS} 次")
    print()

    # ── 建场景 ──────────────────────────────────────────────
    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"            : "step4_Friction_Final",
            "Y_input_range_mm": y_range,
            "Yaw_input_range_deg": yaw_range,
            "Pitch_input_range_deg": pitch_range,
            "MU_SEARCH_S"     : [float(m) for m in MU_SEARCH_S],
            "N_TRIALS"        : N_TRIALS,
            "Z_OFFSET_m"      : Z_OFFSET,
        },
        "trials"  : [],
        "summary" : [],
        "safe_mu_range"  : None,
        "final_safe_pose": {},
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    # ── μ 扫描 ──────────────────────────────────────────────
    rates = []
    for mu_s in MU_SEARCH_S:
        print(f"\n  ── μ_roller_s = {mu_s:.3f}  "
              f"(μ_eff ≈ {mu_s*PALLET_MU_S:.4f}) ──")
        successes = 0
        for t in range(N_TRIALS):
            # Y/Yaw/Pitch 从各自安全区间随机采样
            y_mm  = np.random.uniform(y_range[0],     y_range[1])
            yaw   = np.random.uniform(yaw_range[0],   yaw_range[1])
            pitch = np.random.uniform(pitch_range[0], pitch_range[1])

            rec = await run_one_trial(
                sh,
                y           = float(y_mm / 1000.0),
                yaw_deg     = float(yaw),
                pitch_deg   = float(pitch),
                z_offset    = Z_OFFSET,
                mu_roller_s = float(mu_s),
                tag         = f"μ={mu_s:.3f} Y={y_mm:.1f} Yaw={yaw:.1f} P={pitch:.1f} #{t+1}",
            )
            rec["mu_roller_s"] = round(float(mu_s), 4)
            rec["Y_mm"]        = round(float(y_mm), 1)
            rec["Yaw_deg"]     = round(float(yaw), 1)
            rec["Pitch_deg"]   = round(float(pitch), 1)
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

    # ── 提取 μ 安全区间 ─────────────────────────────────────
    mu_values = [float(m) for m in MU_SEARCH_S]
    safe_mu   = extract_safe_range(mu_values, rates, threshold=0.8)
    log["safe_mu_range"] = safe_mu

    # ── 汇总最终 Safe Pose Set ──────────────────────────────
    log["final_safe_pose"] = {
        "position": {
            "X_m"        : PAL_X0_ACTUAL,
            "Y_range_mm" : y_range,
            "Y_center_m" : PAL_Y0_CENTER,
            "Z_m"        : PAL_Z0_ACTUAL + Z_OFFSET,
            "Z_offset_m" : Z_OFFSET,
        },
        "orientation": {
            "Roll_deg"     : 0.0,
            "Pitch_range_deg": pitch_range,
            "Yaw_range_deg": yaw_range,
        },
        "friction": {
            "mu_roller_s_range": safe_mu,
            "mu_roller_s_nominal": MU_ROLLER_S_BASE,
            "mu_pallet_s"      : PALLET_MU_S,
        },
    }
    save()

    print("\n" + "═" * 62)
    print("  ★ 最终 Safe Pose Set")
    print("═" * 62)
    print(f"    Y       : [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"    Yaw     : [{yaw_range[0]:.1f}, {yaw_range[1]:.1f}]°")
    print(f"    Pitch   : [{pitch_range[0]:.1f}, {pitch_range[1]:.1f}]°")
    print(f"    μ_roller: {safe_mu or '(未收敛，看 summary)'}")
    print(f"\n  ✓ 已写入: {SAVE_PATH}")
    print("  → 此区间可作为 RL 训练目标位姿的采样域")
    print("═" * 62)


asyncio.ensure_future(main())
