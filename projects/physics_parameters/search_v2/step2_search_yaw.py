"""
step2_search_yaw.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 2: 读 safe_pose_y.json 的 Y 安全区间，固定 μ、Pitch=4°，
        对 Yaw 进行域随机化搜索

工作方式：
  • Y 从 Step1 找到的安全区间中随机采样（不再固定在 Y_critical×0.5）
  • Yaw 从 0~8° 逐点搜索
  • μ 同时在 ±30% 范围内随机

输出：projects/physics_parameters/results/safe_pose_yaw.json

使用：在 Isaac Sim Script Editor 粘贴运行
═══════════════════════════════════════════════════════════════════
"""

import os

from pathlib import Path as _ProjectPath

def _physics_project_root():
    candidates = [os.environ.get("ISAAC_OCS_PROJECT_ROOT"), globals().get("__file__"), str(_ProjectPath.cwd())]
    for candidate in candidates:
        if not candidate:
            continue
        start = _ProjectPath(candidate).expanduser().resolve()
        if start.is_file():
            start = start.parent
        for parent in (start, *start.parents):
            if (parent / "project_manifest.yaml").is_file():
                return parent
    raise RuntimeError("Set ISAAC_OCS_PROJECT_ROOT to the project directory before running in Script Editor")

_PHYSICS_ROOT = _physics_project_root() / "projects" / "physics_parameters"


import os
import sys
import json
import asyncio
import numpy as np

COMMON_DIR = str(_PHYSICS_ROOT / "search_v2")
if COMMON_DIR not in sys.path:
    sys.path.insert(0, COMMON_DIR)

from safe_pose_common import (
    SceneHandle, run_one_trial, extract_safe_range,
    RESULT_DIR, MU_ROLLER_S_BASE, MU_ROLLER_BASE, MU_RAND_RANGE,
    PITCH_BASE,
)

# ═══════════════════════════════════════════════════════════════
# Step 2 专属参数
# ═══════════════════════════════════════════════════════════════

YAW_SEARCH = np.linspace(0.0, 8.0, 9)
N_TRIALS   = 5
Y_INPUT    = os.path.join(RESULT_DIR, "safe_pose_y.json")
SAVE_PATH  = os.path.join(RESULT_DIR, "safe_pose_yaw.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 2] Yaw 搜索 · Y 从 Step1 区间随机 · Pitch=4°")
    print("═" * 62)

    # ── 读取 Step1 的 Y 安全区间 ────────────────────────────
    if not os.path.exists(Y_INPUT):
        print(f"✗ 找不到 {Y_INPUT}")
        print("  请先运行 step1_search_y.py")
        return
    with open(Y_INPUT) as f:
        y_log = json.load(f)
    y_range = y_log.get("safe_range_mm")
    if y_range is None:
        print("⚠ Step1 未找到安全区间，用全范围 0~40mm 兜底")
        y_range = [0.0, 40.0]
    print(f"  Y 采样区间: [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"  Yaw 搜索点: {[round(w,1) for w in YAW_SEARCH]}°")
    print(f"  每点重复  : {N_TRIALS} 次")
    print(f"  输出      : {SAVE_PATH}")
    print()

    # ── 建场景句柄 ──────────────────────────────────────────
    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"              : "step2_Yaw",
            "Y_input_range_mm"  : y_range,
            "MU_ROLLER_S_BASE"  : MU_ROLLER_S_BASE,
            "MU_ROLLER_BASE"    : MU_ROLLER_BASE,
            "MU_RAND_RANGE"     : MU_RAND_RANGE,
            "PITCH_BASE"        : PITCH_BASE,
            "YAW_SEARCH_deg"    : [round(w,1) for w in YAW_SEARCH],
            "N_TRIALS"          : N_TRIALS,
        },
        "trials"       : [],
        "summary"      : [],
        "safe_range_deg": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    # ── 逐 Yaw 循环 ──────────────────────────────────────────
    rates = []
    for yaw in YAW_SEARCH:
        print(f"\n  ── Yaw = {yaw:.1f}° ──")
        successes = 0
        for t in range(N_TRIALS):
            # Y 从安全区间随机采样
            y_mm  = np.random.uniform(y_range[0], y_range[1])
            y_val = float(y_mm / 1000.0)
            # μ 域随机化
            mu_eff = MU_ROLLER_BASE * (
                1.0 + np.random.uniform(-MU_RAND_RANGE, MU_RAND_RANGE))
            mu_eff = float(np.clip(mu_eff, 0.005, 0.065))
            mu_s   = mu_eff / 0.400

            rec = await run_one_trial(
                sh,
                y           = y_val,
                yaw_deg     = float(yaw),
                pitch_deg   = PITCH_BASE,
                mu_roller_s = mu_s,
                tag         = f"Yaw={yaw:.1f}° Y={y_mm:.1f}mm #{t+1}/{N_TRIALS}",
            )
            rec["Yaw_deg"] = round(float(yaw), 1)
            rec["Y_mm"]    = round(y_mm, 1)
            rec["mu_eff"]  = round(mu_eff, 4)
            log["trials"].append(rec)
            if rec["result"] == "success":
                successes += 1
            save()

        rate = successes / N_TRIALS
        rates.append(rate)
        status = "✓" if rate >= 0.8 else ("△" if rate >= 0.4 else "✗")
        print(f"  → 成功率 = {rate:.0%}  {status}")

        log["summary"].append({
            "Yaw_deg": round(float(yaw), 1),
            "rate"   : round(rate, 3),
        })
        save()

    # ── 提取安全 Yaw 区间 ───────────────────────────────────
    values_deg = [round(w,1) for w in YAW_SEARCH]
    safe_range = extract_safe_range(values_deg, rates, threshold=0.8)
    log["safe_range_deg"] = safe_range
    save()

    print("\n" + "═" * 62)
    if safe_range:
        print(f"  ✓ Yaw 安全区间: [{safe_range[0]}, {safe_range[1]}]°")
    else:
        print("  ⚠ 未找到 Yaw 安全区间")
    print(f"  ✓ 结果已保存: {SAVE_PATH}")
    print("═" * 62)


asyncio.ensure_future(main())
