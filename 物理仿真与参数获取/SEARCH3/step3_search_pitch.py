"""
step3_search_pitch.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 3: 读 Step1+2 的 Y / Yaw 安全区间，略微抬高 Z（3mm）
        ★ 只对 Pitch 进行扫描（μ、Y、Yaw 都不随机化）

策略：
  • Y 固定为 Step1 区间中点
  • Yaw 固定为 Step2 区间中点
  • 只 Pitch 在变（从 4° 到 25°，步长 3°）
  • 这样 Pitch 的安全区间是"在最代表性的 Y/Yaw 配置下"的结果
  • 每个 Pitch 跑 N_TRIALS 次，仅用于确认可重复性

如果想额外验证 Y×Yaw 组合的 robustness，可在 Step3 之后另写一个
专门的 cross-validation 脚本，不应混在 Pitch 搜索里。

输出：~/Desktop/user_scripts/result/safe_pose_pitch.json
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
    RESULT_DIR, MU_ROLLER_S_BASE, MU_ROLLER_BASE, PITCH_BASE,
)

# ═══════════════════════════════════════════════════════════════
# Step 3 参数
# ═══════════════════════════════════════════════════════════════

PITCH_SEARCH = np.arange(4.0, 25.0 + 0.01, 3.0)   # 4, 7, 10, 13, 16, 19, 22, 25
Z_OFFSET     = 0.003                              # 抬高 3mm
N_TRIALS     = 5

Y_INPUT   = os.path.join(RESULT_DIR, "safe_pose_y.json")
YAW_INPUT = os.path.join(RESULT_DIR, "safe_pose_yaw.json")
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_pitch.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 3] Pitch 扫描")
    print("  ★ 只扫描 Pitch，Y/Yaw 固定为前两步区间中点，μ 固定")
    print("═" * 62)

    # ── 读取前两步的安全区间 ────────────────────────────────
    if not os.path.exists(Y_INPUT) or not os.path.exists(YAW_INPUT):
        print(f"✗ 缺少 step1/step2 的输出，请先依次运行")
        return
    with open(Y_INPUT) as f:
        y_range = json.load(f).get("safe_range_mm") or [0.0, 0.0]
    with open(YAW_INPUT) as f:
        yaw_range = json.load(f).get("safe_range_deg") or [0.0, 0.0]

    # Y, Yaw 都取各自区间的中点
    Y_FIXED_MM   = (y_range[0]   + y_range[1])   / 2.0
    Y_FIXED_M    = Y_FIXED_MM / 1000.0
    YAW_FIXED    = (yaw_range[0] + yaw_range[1]) / 2.0

    print(f"  μ_roller_s   : {MU_ROLLER_S_BASE}（固定）")
    print(f"  Y  区间      : [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"  Y  固定      : {Y_FIXED_MM:.1f}mm")
    print(f"  Yaw 区间     : [{yaw_range[0]:.1f}, {yaw_range[1]:.1f}]°")
    print(f"  Yaw 固定     : {YAW_FIXED:.1f}°")
    print(f"  Pitch 搜索点 : {[round(float(p),1) for p in PITCH_SEARCH]}°")
    print(f"  Z 抬高       : {Z_OFFSET*1000:.1f} mm")
    print(f"  每点重复     : {N_TRIALS} 次")
    print(f"  输出         : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"               : "step3_Pitch",
            "note"               : "只扫描 Pitch，其余固定",
            "MU_ROLLER_S_BASE"   : MU_ROLLER_S_BASE,
            "MU_ROLLER_BASE"     : MU_ROLLER_BASE,
            "Y_input_range_mm"   : y_range,
            "Y_fixed_mm"         : round(Y_FIXED_MM, 1),
            "Yaw_input_range_deg": yaw_range,
            "Yaw_fixed_deg"      : round(YAW_FIXED, 1),
            "PITCH_SEARCH_deg"   : [round(float(p),1) for p in PITCH_SEARCH],
            "Z_OFFSET_m"         : Z_OFFSET,
            "N_TRIALS"           : N_TRIALS,
        },
        "trials"        : [],
        "summary"       : [],
        "safe_range_deg": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for pitch in PITCH_SEARCH:
        print(f"\n  ── Pitch = {pitch:.1f}° ──")
        successes = 0
        for t in range(N_TRIALS):
            rec = await run_one_trial(
                sh,
                y           = Y_FIXED_M,
                yaw_deg     = float(YAW_FIXED),
                pitch_deg   = float(pitch),
                z_offset    = Z_OFFSET,
                mu_roller_s = MU_ROLLER_S_BASE,
                tag         = f"Pitch={pitch:.1f}° #{t+1}/{N_TRIALS}",
            )
            rec["Pitch_deg"]  = round(float(pitch), 1)
            rec["Y_mm"]       = round(Y_FIXED_MM, 1)
            rec["Yaw_deg"]    = round(float(YAW_FIXED), 1)
            rec["Z_offset_m"] = Z_OFFSET
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
            save()

        rate = successes / N_TRIALS
        rates.append(rate)
        status = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  → 成功率 = {rate:.0%}  {status}")

        log["summary"].append({
            "Pitch_deg": round(float(pitch), 1),
            "rate"     : round(rate, 3),
        })
        save()

    values_deg = [round(float(p), 1) for p in PITCH_SEARCH]
    safe_range = extract_safe_range(values_deg, rates, threshold=0.8)
    log["safe_range_deg"] = safe_range
    save()

    print("\n" + "═" * 62)
    if safe_range:
        print(f"  ✓ Pitch 安全区间: [{safe_range[0]}, {safe_range[1]}]°")
    else:
        print("  ⚠ 未找到 Pitch 安全区间")
    print(f"  ✓ 结果已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
