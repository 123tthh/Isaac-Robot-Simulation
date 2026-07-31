"""
step1_search_y.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 1: 固定 μ_roller_s=0.060、Yaw=0、Pitch=4°
        ★ 只对 Y 偏心进行扫描（μ 不随机化）

搜索逻辑：
  • Y 扫描 9 个点（0~40mm）
  • 每个 Y 点跑 N_TRIALS 次，所有参数都固定（μ、Yaw、Pitch）
  • N_TRIALS > 1 仅用于确认仿真可重复性
    （PhysX 解算器有少量非确定性：接触求解顺序、SIMD 浮点等）

输出：~/Desktop/user_scripts/result/safe_pose_y.json
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
# Step 1 参数
# ═══════════════════════════════════════════════════════════════

Y_SEARCH  = np.linspace(0.000, 0.040, 9)   # 0~40mm 共 9 点
N_TRIALS  = 5
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_y.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 1] Y 偏心扫描")
    print("  ★ 只扫描 Y，μ=0.060 / Yaw=0° / Pitch=4° 全部固定")
    print("═" * 62)
    print(f"  μ_roller_s : {MU_ROLLER_S_BASE}  (μ_eff={MU_ROLLER_BASE:.4f})")
    print(f"  Y 搜索点   : {[round(y*1000,1) for y in Y_SEARCH]} mm")
    print(f"  每点重复   : {N_TRIALS} 次（确认可重复性）")
    print(f"  输出       : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"            : "step1_Y",
            "note"            : "只扫描 Y，μ/Yaw/Pitch 全部固定",
            "MU_ROLLER_S_BASE": MU_ROLLER_S_BASE,
            "MU_ROLLER_BASE"  : MU_ROLLER_BASE,
            "PITCH_BASE"      : PITCH_BASE,
            "Yaw_fixed_deg"   : 0.0,
            "Y_SEARCH_mm"     : [round(y*1000,1) for y in Y_SEARCH],
            "N_TRIALS"        : N_TRIALS,
        },
        "trials"       : [],
        "summary"      : [],
        "safe_range_mm": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for y in Y_SEARCH:
        print(f"\n  ── Y = {y*1000:.1f}mm ──")
        successes = 0
        for t in range(N_TRIALS):
            # ★ 不随机 μ，全部用基准值
            rec = await run_one_trial(
                sh,
                y           = float(y),
                yaw_deg     = 0.0,
                pitch_deg   = PITCH_BASE,
                mu_roller_s = MU_ROLLER_S_BASE,   # ← 固定
                tag         = f"Y={y*1000:.1f}mm #{t+1}/{N_TRIALS}",
            )
            rec["Y_mm"] = round(float(y)*1000, 1)
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
            save()

        rate = successes / N_TRIALS
        rates.append(rate)
        status = "✓下滑" if rate >= 0.8 else ("△不稳" if rate >= 0.4 else "✗卡住")
        print(f"  → 成功率 = {rate:.0%}  {status}")

        log["summary"].append({
            "Y_mm": round(float(y)*1000, 1),
            "rate": round(rate, 3),
        })
        save()

    values_mm  = [round(y*1000,1) for y in Y_SEARCH]
    safe_range = extract_safe_range(values_mm, rates, threshold=0.8)
    log["safe_range_mm"] = safe_range
    save()

    print("\n" + "═" * 62)
    if safe_range:
        print(f"  ✓ Y 安全区间: [{safe_range[0]}, {safe_range[1]}] mm")
    else:
        print("  ⚠ 未找到 Y 安全区间")
    print(f"  ✓ 结果已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
