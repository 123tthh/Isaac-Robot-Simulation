"""
step2_search_yaw.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 2: 读 Step1 的 Y 安全区间，固定 μ、Pitch=4°
        ★ 只对 Yaw 进行扫描（μ/Y 都不随机化）

策略：Y 固定为 Step1 安全区间的中点
  • 这样只有 Yaw 在变，结果直接反映 Yaw 对下滑的影响
  • 比"Y 从区间随机"更干净，避免 Y 和 Yaw 的方差混淆

关于 Q3（Y 和 Yaw 的耦合）：
  串行搜索的理论缺陷是"某些不安全 Y 可能在特定 Yaw 下变安全"。
  这属于二阶耦合项（∝ Y × sin(Yaw)），工程上忽略即可：
  (1) RL 训练更偏向 robust 中心区域
  (2) Step3 在 Y×Yaw 小网格上重测会自然覆盖耦合
  (3) 真正解耦需要 6D 全空间搜索，计算代价不值

输出：projects/physics_parameters/results/safe_pose_yaw.json
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

COMMON_DIR = str(_PHYSICS_ROOT / "search_v3")
if COMMON_DIR not in sys.path:
    sys.path.insert(0, COMMON_DIR)

from safe_pose_common import (
    SceneHandle, run_one_trial, extract_safe_range,
    RESULT_DIR, MU_ROLLER_S_BASE, MU_ROLLER_BASE, PITCH_BASE,
)

# ═══════════════════════════════════════════════════════════════
# Step 2 参数
# ═══════════════════════════════════════════════════════════════

YAW_SEARCH = np.linspace(0.0, 8.0, 9)
N_TRIALS   = 5
Y_INPUT    = os.path.join(RESULT_DIR, "safe_pose_y.json")
SAVE_PATH  = os.path.join(RESULT_DIR, "safe_pose_yaw.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 2] Yaw 扫描")
    print("  ★ 只扫描 Yaw，Y 固定为 Step1 区间中点，μ/Pitch 固定")
    print("═" * 62)

    # ── 读取 Step1 的 Y 安全区间 ────────────────────────────
    if not os.path.exists(Y_INPUT):
        print(f"✗ 找不到 {Y_INPUT}，请先运行 step1_search_y.py")
        return
    with open(Y_INPUT) as f:
        y_log = json.load(f)
    y_range = y_log.get("safe_range_mm")
    if y_range is None:
        print("⚠ Step1 未找到安全区间，回退用 Y=0mm")
        y_range = [0.0, 0.0]

    # Y 固定为区间中点（不做随机，避免引入噪声）
    Y_FIXED_MM = (y_range[0] + y_range[1]) / 2.0
    Y_FIXED_M  = Y_FIXED_MM / 1000.0

    print(f"  μ_roller_s : {MU_ROLLER_S_BASE}（固定）")
    print(f"  Y 区间     : [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"  Y 固定值   : {Y_FIXED_MM:.1f}mm（区间中点）")
    print(f"  Yaw 搜索点 : {[round(w,1) for w in YAW_SEARCH]}°")
    print(f"  每点重复   : {N_TRIALS} 次")
    print(f"  输出       : {SAVE_PATH}\n")

    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"            : "step2_Yaw",
            "note"            : "只扫描 Yaw，其余固定",
            "MU_ROLLER_S_BASE": MU_ROLLER_S_BASE,
            "MU_ROLLER_BASE"  : MU_ROLLER_BASE,
            "PITCH_BASE"      : PITCH_BASE,
            "Y_input_range_mm": y_range,
            "Y_fixed_mm"      : round(Y_FIXED_MM, 1),
            "YAW_SEARCH_deg"  : [round(w,1) for w in YAW_SEARCH],
            "N_TRIALS"        : N_TRIALS,
        },
        "trials"        : [],
        "summary"       : [],
        "safe_range_deg": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    rates = []
    for yaw in YAW_SEARCH:
        print(f"\n  ── Yaw = {yaw:.1f}° ──")
        successes = 0
        for t in range(N_TRIALS):
            # ★ 所有参数固定，只 Yaw 在变
            rec = await run_one_trial(
                sh,
                y           = Y_FIXED_M,
                yaw_deg     = float(yaw),
                pitch_deg   = PITCH_BASE,
                mu_roller_s = MU_ROLLER_S_BASE,
                tag         = f"Yaw={yaw:.1f}° #{t+1}/{N_TRIALS}",
            )
            rec["Yaw_deg"] = round(float(yaw), 1)
            rec["Y_mm"]    = round(Y_FIXED_MM, 1)
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

    values_deg = [round(w, 1) for w in YAW_SEARCH]
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
