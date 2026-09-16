"""
step3_search_pitch.py  ── Isaac Sim 5.1 · Script Editor
═══════════════════════════════════════════════════════════════════
Step 3: 读 Step1+2 的 Y / Yaw 安全区间，略微抬高 Z（<5mm），
        对 Pitch 从 4° 开始递增搜索（<25°），找到临界 Pitch

工作方式：
  • 构造 Y × Yaw 小网格（3×3 = 9 格），每格跑多组 Pitch
  • Pitch 从 PITCH_MIN 递增到 PITCH_MAX，每个 Pitch 重复 N_TRIALS 次
  • Y/Yaw 从前两步安全区间中采样，μ 仍做 ±30% 域随机化

输出：projects/physics_parameters/results/safe_pose_pitch.json

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
# Step 3 专属参数
# ═══════════════════════════════════════════════════════════════

PITCH_SEARCH = np.arange(4.0, 25.0 + 0.01, 3.0)   # 4, 7, 10, 13, 16, 19, 22, 25
Z_OFFSET     = 0.003                               # 抬高 3mm（<5mm）
N_TRIALS     = 3                                   # 每点重复
N_Y_SAMPLES  = 3                                   # Y×Yaw 小网格 3×3
N_YAW_SAMPLES = 3

Y_INPUT   = os.path.join(RESULT_DIR, "safe_pose_y.json")
YAW_INPUT = os.path.join(RESULT_DIR, "safe_pose_yaw.json")
SAVE_PATH = os.path.join(RESULT_DIR, "safe_pose_pitch.json")


async def main():
    os.makedirs(RESULT_DIR, exist_ok=True)
    print("═" * 62)
    print("[Step 3] Pitch 搜索 · Y×Yaw 网格 · Z+3mm")
    print("═" * 62)

    # ── 读取前两步结果 ──────────────────────────────────────
    if not os.path.exists(Y_INPUT) or not os.path.exists(YAW_INPUT):
        print(f"✗ 缺少前置文件，请先运行 step1 和 step2")
        return
    with open(Y_INPUT) as f:
        y_range = json.load(f).get("safe_range_mm") or [0.0, 40.0]
    with open(YAW_INPUT) as f:
        yaw_range = json.load(f).get("safe_range_deg") or [0.0, 8.0]

    print(f"  Y 区间   : [{y_range[0]:.1f}, {y_range[1]:.1f}] mm")
    print(f"  Yaw 区间 : [{yaw_range[0]:.1f}, {yaw_range[1]:.1f}]°")
    print(f"  Pitch 搜: {list(PITCH_SEARCH)}°")
    print(f"  Z 抬高  : {Z_OFFSET*1000:.1f} mm")
    print(f"  每格跑  : N_TRIALS={N_TRIALS} 次 × {len(PITCH_SEARCH)} 个 Pitch")
    print()

    # ── Y × Yaw 采样点（3×3 = 9 格）──────────────────────────
    y_samples   = np.linspace(y_range[0],  y_range[1],  N_Y_SAMPLES)     # mm
    yaw_samples = np.linspace(yaw_range[0], yaw_range[1], N_YAW_SAMPLES) # °

    # ── 建场景 ──────────────────────────────────────────────
    sh = SceneHandle()
    await sh.setup()

    log = {
        "config": {
            "step"              : "step3_Pitch",
            "Y_input_range_mm"  : y_range,
            "Yaw_input_range_deg": yaw_range,
            "PITCH_SEARCH_deg"  : [float(p) for p in PITCH_SEARCH],
            "Z_OFFSET_m"        : Z_OFFSET,
            "Y_samples_mm"      : [round(float(y),1) for y in y_samples],
            "Yaw_samples_deg"   : [round(float(w),1) for w in yaw_samples],
            "N_TRIALS"          : N_TRIALS,
            "MU_ROLLER_S_BASE"  : MU_ROLLER_S_BASE,
            "MU_RAND_RANGE"     : MU_RAND_RANGE,
        },
        "trials"    : [],
        "pitch_summary": [],    # 每个 Pitch 在全网格的平均成功率
        "grid"      : [],       # 每个 (Y,Yaw,Pitch) 三元组的结果
        "safe_range_deg": None,
    }

    def save():
        with open(SAVE_PATH, "w") as f:
            json.dump(log, f, indent=2, ensure_ascii=False)

    # ── 主循环：Pitch 外层，Y×Yaw 内层 ───────────────────────
    pitch_overall_rates = []

    for pitch in PITCH_SEARCH:
        print(f"\n  ══ Pitch = {pitch:.1f}° ══")
        pitch_success = 0
        pitch_total   = 0

        for yi, y_mm in enumerate(y_samples):
            for wi, yaw in enumerate(yaw_samples):
                cell_success = 0
                for t in range(N_TRIALS):
                    # μ 随机化
                    mu_eff = MU_ROLLER_BASE * (
                        1.0 + np.random.uniform(-MU_RAND_RANGE, MU_RAND_RANGE))
                    mu_eff = float(np.clip(mu_eff, 0.005, 0.065))
                    mu_s   = mu_eff / 0.400

                    rec = await run_one_trial(
                        sh,
                        y           = float(y_mm / 1000.0),
                        yaw_deg     = float(yaw),
                        pitch_deg   = float(pitch),
                        z_offset    = Z_OFFSET,
                        mu_roller_s = mu_s,
                        tag         = f"P={pitch:.0f}° Y={y_mm:.1f} Yaw={yaw:.1f} #{t+1}",
                    )
                    rec["Pitch_deg"] = round(float(pitch), 1)
                    rec["Y_mm"]      = round(float(y_mm), 1)
                    rec["Yaw_deg"]   = round(float(yaw), 1)
                    rec["Z_offset_m"]= Z_OFFSET
                    rec["mu_eff"]    = round(mu_eff, 4)
                    log["trials"].append(rec)
                    if rec["result"] == "success":
                        cell_success += 1
                        pitch_success += 1
                    pitch_total += 1
                    save()

                log["grid"].append({
                    "Pitch_deg": round(float(pitch), 1),
                    "Y_mm"     : round(float(y_mm), 1),
                    "Yaw_deg"  : round(float(yaw), 1),
                    "rate"     : round(cell_success / N_TRIALS, 3),
                })

        pitch_rate = pitch_success / pitch_total if pitch_total else 0.0
        pitch_overall_rates.append(pitch_rate)
        log["pitch_summary"].append({
            "Pitch_deg": round(float(pitch), 1),
            "rate"     : round(pitch_rate, 3),
        })
        save()
        status = "✓" if pitch_rate >= 0.8 else ("△" if pitch_rate >= 0.4 else "✗")
        print(f"  Pitch={pitch:.1f}° 总成功率 = {pitch_rate:.0%}  {status}")

    # ── 提取 Pitch 安全区间 ─────────────────────────────────
    values_deg = [round(float(p), 1) for p in PITCH_SEARCH]
    safe_range = extract_safe_range(values_deg, pitch_overall_rates, threshold=0.7)
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
