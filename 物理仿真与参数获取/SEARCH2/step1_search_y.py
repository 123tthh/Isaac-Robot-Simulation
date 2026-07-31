"""
step1_search_y.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 1: 固定 μ、Yaw=0、Pitch=4°，只对 Y 偏心进行域随机化搜索

工作方式：
  • 所有 trial 全自动执行，不需要手动点 Play
  • 每个 Y 值跑 N_TRIALS 次，摩擦在 ±30% 范围内随机
  • 全部结果（含每次 trial 的 final_x / final_v）写入 safe_pose_y.json

输出：~/Desktop/user_scripts/result/safe_pose_y.json

前置：把 safe_pose_common.py 放在 ~/Desktop/user_scripts/
使用：在 Isaac Sim Script Editor 粘贴运行
═══════════════════════════════════════════════════════════════════
"""

import os
import sys
import json
import asyncio
import numpy as np

# 让脚本能找到 safe_pose_common.py
COMMON_DIR = os.path.expanduser("~/Desktop/user_scripts")
if COMMON_DIR not in sys.path:
    sys.path.insert(0, COMMON_DIR)

from safe_pose_common import (
    SceneHandle, run_one_trial, extract_safe_range,
    RESULT_DIR, MU_ROLLER_S_BASE, MU_ROLLER_BASE, MU_RAND_RANGE,
    PITCH_BASE, PAL_Y0_CENTER,
)

# ═══════════════════════════════════════════════════════════════
# Step 1 专属参数
# ═══════════════════════════════════════════════════════════════

Y_SEARCH  = np.linspace(0.000, 0.040, 9)   # 0~40mm 共 9 点
N_TRIALS  = 5                              # 每点重复次数
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_y.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 1] Y 偏心搜索 · Yaw=0 Pitch=4° μ=0.024（随机±30%）")
    print("═" * 62)
    print(f"  Y 搜索点: {[round(y*1000,1) for y in Y_SEARCH]} mm")
    print(f"  每点重复: {N_TRIALS} 次")
    print(f"  输出     : {SAVE_PATH}")
    print()

    # ── 建场景句柄（只做一次）──────────────────────────────
    sh = SceneHandle()
    await sh.setup()

    # ── 日志结构 ────────────────────────────────────────────
    log = {
        "config": {
            "step"              : "step1_Y",
            "MU_ROLLER_S_BASE"  : MU_ROLLER_S_BASE,
            "MU_ROLLER_BASE"    : MU_ROLLER_BASE,
            "MU_RAND_RANGE"     : MU_RAND_RANGE,
            "PITCH_BASE"        : PITCH_BASE,
            "Y_SEARCH_mm"       : [round(y*1000,1) for y in Y_SEARCH],
            "N_TRIALS"          : N_TRIALS,
        },
        "trials"    : [],   # 每一次 trial
        "summary"   : [],   # 每个 Y 的成功率
        "safe_range_mm": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    # ── 逐 Y 循环 ─────────────────────────────────────────
    rates = []
    for y in Y_SEARCH:
        print(f"\n  ── Y = {y*1000:.1f}mm ──")
        successes = 0
        for t in range(N_TRIALS):
            # μ 域随机化
            mu_eff = MU_ROLLER_BASE * (
                1.0 + np.random.uniform(-MU_RAND_RANGE, MU_RAND_RANGE))
            mu_eff = float(np.clip(mu_eff, 0.005, 0.065))
            mu_s   = mu_eff / 0.400   # Roller 的 μs

            rec = await run_one_trial(
                sh,
                y           = float(y),
                yaw_deg     = 0.0,
                pitch_deg   = PITCH_BASE,
                mu_roller_s = mu_s,
                tag         = f"Y={y*1000:.1f}mm #{t+1}/{N_TRIALS}",
            )
            rec["Y_mm"]    = round(float(y)*1000, 1)
            rec["mu_eff"]  = round(mu_eff, 4)
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
            save()   # 每次 trial 后增量保存

        rate = successes / N_TRIALS
        rates.append(rate)
        status = "✓下滑" if rate >= 0.8 else ("△不稳" if rate >= 0.4 else "✗卡住")
        print(f"  → 成功率 = {rate:.0%}  {status}")

        log["summary"].append({
            "Y_mm": round(float(y)*1000, 1),
            "rate": round(rate, 3),
        })
        save()

    # ── 提取连续安全区间 ────────────────────────────────────
    values_mm = [round(y*1000,1) for y in Y_SEARCH]
    safe_range = extract_safe_range(values_mm, rates, threshold=0.8)
    log["safe_range_mm"] = safe_range
    save()

    print("\n" + "═" * 62)
    if safe_range:
        print(f"  ✓ Y 安全区间: [{safe_range[0]}, {safe_range[1]}] mm")
        print(f"    （成功率≥80% 的连续最长区间）")
    else:
        print("  ⚠ 未找到成功率≥80% 的连续区间")
        print("    查看 summary 字段，按实际情况放宽阈值或调整摩擦")
    print(f"\n  ✓ 完整结果已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
