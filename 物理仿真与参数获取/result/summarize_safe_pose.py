"""
summarize_safe_pose.py
═══════════════════════════════════════════════════════════════════
汇总 step1~step5 的 JSON 结果，生成最终的夹爪释放参数空间。

输入：~/Desktop/user_scripts/result/{safe_pose_y, safe_pose_yaw,
       safe_pose_pitch, safe_pose_mass, safe_pose}.json
输出：~/Desktop/user_scripts/result/final_release_space.json
       以及控制台打印 + 可选的 markdown 报告

★ 两种区间模式：
  1. naive    : 直接取各 JSON 的 safe_range_* 字段（快，但受孤立点/空洞影响）
  2. robust   : 重新分析 summary，只保留严格连续的 rate≥threshold 区间
                （排除锯齿、孤立成功点等仿真噪声）

  对于 RL 训练采样域，推荐用 robust。

★ 每个参数都提供 conservative / nominal / full 三档：
  - conservative : robust 连续区间的 60% 居中子集（训练 RL 的目标核心区）
  - nominal      : robust 连续区间本身（能跑通的安全区）
  - full         : 扫描边界（作为 RL 课程学习的最外圈）
═══════════════════════════════════════════════════════════════════
"""

import os
import json
import sys
import math
from datetime import datetime

# ─── 输入输出路径（可调整）──────────────────────────────────
RESULT_DIR = os.path.expanduser("~/Desktop/user_scripts/result")
OUTPUT_JSON = os.path.join(RESULT_DIR, "final_release_space.json")
OUTPUT_MD   = os.path.join(RESULT_DIR, "final_release_space.md")

INPUT_FILES = {
    "y"       : "safe_pose_y.json",
    "yaw"     : "safe_pose_yaw.json",
    "pitch"   : "safe_pose_pitch.json",
    "mass"    : "safe_pose_mass.json",
    "friction": "safe_pose.json",
}

# ─── 分析参数 ───────────────────────────────────────────────
RATE_THRESHOLD = 0.8      # 判为"安全"的成功率阈值
CONSERVATIVE_SHRINK = 0.60  # conservative = nominal 居中的 60%

# 固定位置（不搜索的参数）
X_FIXED_M        = 0.045
ROLL_FIXED_DEG   = 0.0
PAL_Y0_CENTER_M  = 0.0
PAL_Z0_ACTUAL_M  = 1.254
PALLET_DIM_X_M   = 0.185
PITCH_BASE_DEG   = 4.0
Z_BASE_M         = 0.00124


# ═══════════════════════════════════════════════════════════════
# 工具
# ═══════════════════════════════════════════════════════════════

def load_json(path):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)


def find_isolated_failures(values_rates, threshold):
    """
    查找"孤立失败点"——前后都成功但自己失败的点。
    这些是 sim2real 风险信号。
    返回 [(v, rate), ...]
    """
    if len(values_rates) < 3:
        return []
    sorted_vr = sorted(values_rates, key=lambda x: x[0])
    isolated = []
    for i in range(1, len(sorted_vr) - 1):
        v, r = sorted_vr[i]
        prev_r = sorted_vr[i-1][1]
        next_r = sorted_vr[i+1][1]
        if r < threshold and prev_r >= threshold and next_r >= threshold:
            isolated.append((v, r))
    return isolated


def extract_longest_contiguous(values_rates, threshold):
    """
    从 [(v, rate), ...] 中找最长的严格连续段：相邻采样点都 rate ≥ threshold，
    一旦遇到 rate < threshold 的点就截断。

    返回 (v_min, v_max) 或 None（没有 ≥2 个连续点）。
    """
    if not values_rates:
        return None
    sorted_vr = sorted(values_rates, key=lambda x: x[0])

    # 扫一遍，记录每一段连续 pass 的 (start_idx, end_idx)
    segments = []
    cur_start = None
    for i, (v, r) in enumerate(sorted_vr):
        if r >= threshold:
            if cur_start is None:
                cur_start = i
            cur_end = i
        else:
            if cur_start is not None:
                segments.append((cur_start, cur_end))
                cur_start = None
    if cur_start is not None:
        segments.append((cur_start, cur_end))

    if not segments:
        return None

    # 选最长段（按点数）
    best = max(segments, key=lambda se: se[1] - se[0])
    lo_v = sorted_vr[best[0]][0]
    hi_v = sorted_vr[best[1]][0]
    # 至少 2 个点
    if best[1] - best[0] < 1:
        return None
    return (float(lo_v), float(hi_v))


def shrink_range(rng, factor):
    """把区间居中收缩到 factor 比例。例如 factor=0.6 → 保留中心 60%。"""
    if rng is None:
        return None
    lo, hi = rng
    center = (lo + hi) / 2.0
    half = (hi - lo) / 2.0 * factor
    return (center - half, center + half)


def pitch_to_z_offset(pitch_deg):
    """与 step3 的 pitch_z_offset 公式一致。"""
    delta = max(0.0, pitch_deg - PITCH_BASE_DEG)
    return Z_BASE_M + math.tan(math.radians(delta)) * (PALLET_DIM_X_M / 2.0)


def format_range(rng, unit="", prec=2):
    if rng is None:
        return "None"
    return f"[{rng[0]:.{prec}f}, {rng[1]:.{prec}f}] {unit}".strip()


# ═══════════════════════════════════════════════════════════════
# 各 step 的分析逻辑
# ═══════════════════════════════════════════════════════════════

def analyze_y(y_data):
    """解析 step1 的 Y 扫描结果。"""
    if not y_data:
        return None
    summary = y_data.get("summary", [])
    vr = [(s["Y_mm"], s["rate"]) for s in summary]
    naive = y_data.get("safe_range_mm")
    robust = extract_longest_contiguous(vr, RATE_THRESHOLD)
    isolated = find_isolated_failures(vr, RATE_THRESHOLD)
    total_lock = sum(s.get("n_rail_lock", 0) for s in summary)
    total_stuck = sum(s.get("n_stuck", 0) for s in summary)
    total_success = sum(s.get("n_success", 0) for s in summary)
    return {
        "unit"        : "mm",
        "param"       : "Y",
        "points"      : len(vr),
        "naive_range" : naive,
        "robust_range": list(robust) if robust else None,
        "conservative": list(shrink_range(robust, CONSERVATIVE_SHRINK)) if robust else None,
        "full_range"  : [vr[0][0], vr[-1][0]] if vr else None,
        "isolated_failures": isolated,
        "total_success": total_success,
        "total_rail_lock": total_lock,
        "total_stuck" : total_stuck,
        "raw"         : vr,
    }


def analyze_yaw(yaw_data):
    if not yaw_data:
        return None
    summary = yaw_data.get("summary", [])
    vr = [(s["Yaw_deg"], s["rate"]) for s in summary]
    naive = yaw_data.get("safe_range_deg")
    robust = extract_longest_contiguous(vr, RATE_THRESHOLD)
    isolated = find_isolated_failures(vr, RATE_THRESHOLD)
    total_lock = sum(s.get("n_rail_lock", 0) for s in summary)
    total_stuck = sum(s.get("n_stuck", 0) for s in summary)
    total_success = sum(s.get("n_success", 0) for s in summary)
    return {
        "unit"        : "deg",
        "param"       : "Yaw",
        "points"      : len(vr),
        "naive_range" : naive,
        "robust_range": list(robust) if robust else None,
        "conservative": list(shrink_range(robust, CONSERVATIVE_SHRINK)) if robust else None,
        "full_range"  : [vr[0][0], vr[-1][0]] if vr else None,
        "isolated_failures": isolated,
        "total_success": total_success,
        "total_rail_lock": total_lock,
        "total_stuck" : total_stuck,
        "raw"         : vr,
    }


def analyze_pitch(pitch_data):
    if not pitch_data:
        return None
    summary = pitch_data.get("summary", [])
    vr = [(s["Pitch_deg"], s["rate"]) for s in summary]
    naive = pitch_data.get("safe_range_deg")
    robust = extract_longest_contiguous(vr, RATE_THRESHOLD)
    isolated = find_isolated_failures(vr, RATE_THRESHOLD)
    total_lock = sum(s.get("n_rail_lock", 0) for s in summary)
    total_stuck = sum(s.get("n_stuck", 0) for s in summary)
    total_success = sum(s.get("n_success", 0) for s in summary)
    return {
        "unit"        : "deg",
        "param"       : "Pitch",
        "points"      : len(vr),
        "naive_range" : naive,
        "robust_range": list(robust) if robust else None,
        "conservative": list(shrink_range(robust, CONSERVATIVE_SHRINK)) if robust else None,
        "full_range"  : [vr[0][0], vr[-1][0]] if vr else None,
        "isolated_failures": isolated,
        "total_success": total_success,
        "total_rail_lock": total_lock,
        "total_stuck" : total_stuck,
        "raw"         : vr,
    }


def analyze_mass(mass_data):
    if not mass_data:
        return None
    summary = mass_data.get("summary", [])
    vr = [(s["Mass_kg"], s["rate"]) for s in summary]
    naive = mass_data.get("safe_mass_range_kg")
    robust = extract_longest_contiguous(vr, RATE_THRESHOLD)
    isolated = find_isolated_failures(vr, RATE_THRESHOLD)
    return {
        "unit"        : "kg",
        "param"       : "Mass",
        "points"      : len(vr),
        "naive_range" : naive,
        "robust_range": list(robust) if robust else None,
        "conservative": list(shrink_range(robust, CONSERVATIVE_SHRINK)) if robust else None,
        "full_range"  : [vr[0][0], vr[-1][0]] if vr else None,
        "isolated_failures": isolated,
        "raw"         : vr,
    }


def analyze_friction(fric_data):
    if not fric_data:
        return None
    summary = fric_data.get("summary", [])
    vr = [(s["mu_roller_s"], s["rate"]) for s in summary]
    naive = fric_data.get("safe_mu_s_range")
    robust = extract_longest_contiguous(vr, RATE_THRESHOLD)
    isolated = find_isolated_failures(vr, RATE_THRESHOLD)
    return {
        "unit"        : "",
        "param"       : "mu_roller_s",
        "points"      : len(vr),
        "naive_range" : naive,
        "robust_range": list(robust) if robust else None,
        "conservative": list(shrink_range(robust, CONSERVATIVE_SHRINK)) if robust else None,
        "full_range"  : [vr[0][0], vr[-1][0]] if vr else None,
        "isolated_failures": isolated,
        "raw"         : vr,
    }


# ═══════════════════════════════════════════════════════════════
# 主流程
# ═══════════════════════════════════════════════════════════════

def main():
    print("═" * 72)
    print("汇总 step1~step5 搜索结果 → 最终参数空间")
    print("═" * 72)

    # 1. 加载所有 JSON
    data = {}
    for key, fname in INPUT_FILES.items():
        path = os.path.join(RESULT_DIR, fname)
        d = load_json(path)
        if d is None:
            print(f"  ⚠ 找不到 {path}")
        else:
            print(f"  ✓ 加载 {fname}")
        data[key] = d

    # 2. 分析各维度
    analysis = {
        "Y"          : analyze_y(data.get("y")),
        "Yaw"        : analyze_yaw(data.get("yaw")),
        "Pitch"      : analyze_pitch(data.get("pitch")),
        "Mass"       : analyze_mass(data.get("mass")),
        "mu_roller_s": analyze_friction(data.get("friction")),
    }

    print()
    print("─" * 72)
    print("各维度分析（naive = JSON里的safe_range，robust = 严格连续子集）")
    print("─" * 72)
    for name, a in analysis.items():
        if a is None:
            print(f"  {name:14s} : 数据缺失")
            continue
        print(f"\n  {name:14s} ({a['unit']:4s}):  共 {a['points']} 点, "
              f"扫描范围 {format_range(a['full_range'], a['unit'])}")
        print(f"    naive        : {format_range(a['naive_range'], a['unit'])}")
        print(f"    robust       : {format_range(a['robust_range'], a['unit'])}")
        print(f"    conservative : {format_range(a['conservative'], a['unit'])}  "
              f"(center ± {CONSERVATIVE_SHRINK*50:.0f}%)")
        if "total_rail_lock" in a:
            print(f"    失败统计     : success={a['total_success']}, "
                  f"rail_lock={a['total_rail_lock']}, stuck={a['total_stuck']}")

        # 孤立失败点诊断
        iso = a.get("isolated_failures", [])
        if iso:
            iso_str = ", ".join(f"{v:+.1f}{a['unit']}" for v, _ in iso)
            print(f"    ⚠ 孤立失败点: {iso_str}  "
                  f"(前后都成功但此点失败 → sim2real 风险)")

        # 给出诊断提示
        if a['naive_range'] and a['robust_range'] and \
           (a['naive_range'][0] != a['robust_range'][0] or
            a['naive_range'][1] != a['robust_range'][1]):
            print(f"    ⚠ naive ≠ robust: naive 含空洞, robust 更可靠")

        if a['robust_range'] == a['full_range']:
            print(f"    ⚠ robust 区间 = 扫描全域: 边界未找到，建议扩大扫描范围")

    # 3. 生成最终位姿区间（两种模式）
    def build_pose_set(mode):
        """mode ∈ {'conservative', 'robust', 'full'}"""
        def get(a, which="robust_range"):
            if a is None:
                return None
            return a.get(which)

        key = {
            "conservative": "conservative",
            "nominal"     : "robust_range",
            "full"        : "full_range",
        }[mode]

        y_rng     = get(analysis["Y"], key)
        yaw_rng   = get(analysis["Yaw"], key)
        pitch_rng = get(analysis["Pitch"], key)
        mass_rng  = get(analysis["Mass"], key)
        mu_rng    = get(analysis["mu_roller_s"], key)

        # Z 是 Pitch 的函数。给出 z 的范围（由 pitch 范围推出）
        if pitch_rng:
            z_lo = pitch_to_z_offset(pitch_rng[0])
            z_hi = pitch_to_z_offset(pitch_rng[1])
            z_range_abs = [PAL_Z0_ACTUAL_M + z_lo, PAL_Z0_ACTUAL_M + z_hi]
        else:
            z_range_abs = [PAL_Z0_ACTUAL_M + Z_BASE_M, PAL_Z0_ACTUAL_M + Z_BASE_M]

        return {
            "position": {
                "X_m"          : X_FIXED_M,
                "Y_range_m"    : [y_rng[0]/1000.0, y_rng[1]/1000.0] if y_rng else None,
                "Y_range_mm"   : y_rng,
                "Y_center_m"   : PAL_Y0_CENTER_M,
                "Z_range_m"    : z_range_abs,
                "Z_center_m"   : PAL_Z0_ACTUAL_M,
                "Z_is_function_of_pitch": True,
                "Z_offset_formula": "Z_BASE + tan(pitch - 4°) * PALLET_DIM_X / 2",
            },
            "orientation": {
                "Roll_deg"       : ROLL_FIXED_DEG,
                "Pitch_range_deg": pitch_rng,
                "Yaw_range_deg"  : yaw_rng,
            },
            "physics": {
                "mass_range_kg"     : mass_rng,
                "mu_roller_s_range" : mu_rng,
                "pallet_dim_m"      : [PALLET_DIM_X_M, 0.265, 0.040],
            },
        }

    # ─── 3.5 Sim2Real 修正 ────────────────────────────────────
    #
    # 基于已知的 sim/real 偏差对 conservative 区间做进一步收紧。
    # 这些是工程判断，不是自动算出来的。
    #
    # ★ 修正 1: Y 可行域 — 真机滚轮摩擦远大于 sim 标定值
    #   sim 里 μ_roller=0.06 标定的等效摩擦偏低，真实滚轮表面磨损/
    #   粗糙度使得实际 μ_roller 更高。高摩擦 → 偏心后侧向分力不够
    #   克服接触力 → 更容易卡住 → Y 安全域缩小。
    #   保守策略: 在 sim conservative 基础上再收缩到 50%。
    #
    SIM2REAL_Y_SHRINK = 0.50   # ← 可调：0.5 = conservative 的中心 50%
    #
    # ★ 修正 2: Pitch 实际工况上限 — 夹爪释放不超过 15°
    #   sim 里 [4, 25]° 全通过，但实际操作工况 Pitch ≤ 15°。
    #   直接 clip 到 [4, 15]°。
    #
    PITCH_REAL_MAX = 15.0
    #
    # ★ 修正 3: Yaw — sim2real gap 最敏感的维度
    #   真实导轨摩擦不确定性最大，sim 里 Yaw ±10° 能过但 real 可能
    #   ±5° 就锁。对 Yaw 额外收缩到 70%。
    #
    SIM2REAL_YAW_SHRINK = 0.70

    def apply_sim2real(pose_set):
        """在 conservative 基础上叠加 sim2real 修正。"""
        ps = json.loads(json.dumps(pose_set))  # deep copy

        # Y 收缩
        y_mm = ps["position"].get("Y_range_mm")
        if y_mm:
            y_shrunk = list(shrink_range(tuple(y_mm), SIM2REAL_Y_SHRINK))
            ps["position"]["Y_range_mm"] = y_shrunk
            ps["position"]["Y_range_m"] = [y_shrunk[0]/1000.0, y_shrunk[1]/1000.0]

        # Pitch clip
        p_rng = ps["orientation"].get("Pitch_range_deg")
        if p_rng:
            ps["orientation"]["Pitch_range_deg"] = [
                p_rng[0],
                min(p_rng[1], PITCH_REAL_MAX)
            ]
            # 同步更新 Z range
            new_p = ps["orientation"]["Pitch_range_deg"]
            z_lo = pitch_to_z_offset(new_p[0])
            z_hi = pitch_to_z_offset(new_p[1])
            ps["position"]["Z_range_m"] = [PAL_Z0_ACTUAL_M + z_lo,
                                           PAL_Z0_ACTUAL_M + z_hi]

        # Yaw 收缩
        yaw_rng = ps["orientation"].get("Yaw_range_deg")
        if yaw_rng:
            ps["orientation"]["Yaw_range_deg"] = list(
                shrink_range(tuple(yaw_rng), SIM2REAL_YAW_SHRINK))

        return ps

    sim2real_pose = apply_sim2real(build_pose_set("conservative"))

    # sim2real 修正说明
    sim2real_notes = {
        "description": "在 conservative 基础上叠加已知 sim/real 偏差的修正",
        "adjustments": [
            {
                "param"  : "Y",
                "reason" : "真机滚轮摩擦远大于 sim 标定值 (μ_sim=0.06)，"
                           "偏心后更容易被摩擦力卡住",
                "method" : f"conservative 区间中心再收缩到 {SIM2REAL_Y_SHRINK*100:.0f}%",
            },
            {
                "param"  : "Pitch",
                "reason" : f"真实夹爪释放工况 Pitch ≤ {PITCH_REAL_MAX}°",
                "method" : f"upper clip 到 {PITCH_REAL_MAX}°",
            },
            {
                "param"  : "Yaw",
                "reason" : "导轨摩擦不确定性大，sim 里 ±10° 通过但 real 可能"
                           " ±5° 就锁死",
                "method" : f"conservative 区间中心再收缩到 {SIM2REAL_YAW_SHRINK*100:.0f}%",
            },
        ],
    }

    final = {
        "meta": {
            "generated_at"   : datetime.now().isoformat(),
            "rate_threshold" : RATE_THRESHOLD,
            "conservative_shrink": CONSERVATIVE_SHRINK,
            "sim2real_notes" : sim2real_notes,
            "notes"          : [
                "Y/Yaw 搜索含锯齿/孤立点，robust 区间比 naive 更保守",
                "Pitch/Mass/μ 搜索全域成功，已按实际工况 clip",
                "Z 是 Pitch 的函数（托盘几何贴合补偿）",
                "Roll 固定为 0°（保持水平）",
                "X 固定为 0.045m（输送机入口）",
                f"sim2real: Y 额外收缩 {SIM2REAL_Y_SHRINK*100:.0f}% "
                f"(真机 μ_roller > sim)",
                f"sim2real: Pitch clip 到 {PITCH_REAL_MAX}°",
                f"sim2real: Yaw 额外收缩 {SIM2REAL_YAW_SHRINK*100:.0f}%",
            ],
        },
        "analysis_per_dim": analysis,
        "release_pose_set": {
            "sim2real"    : sim2real_pose,
            "conservative": build_pose_set("conservative"),
            "nominal"     : build_pose_set("nominal"),
            "full"        : build_pose_set("full"),
        },
    }

    # 4. 保存 JSON
    os.makedirs(RESULT_DIR, exist_ok=True)
    with open(OUTPUT_JSON, "w") as f:
        json.dump(final, f, indent=2, ensure_ascii=False)

    # 5. 控制台打印最终结果
    print()
    print("═" * 72)
    print("最终释放位姿参数空间（RL 训练目标）")
    print("═" * 72)

    for mode, label in [("sim2real",     "★ Sim2Real 推荐区（真机部署用）"),
                        ("conservative", "保守区（sim 内训练核心）"),
                        ("nominal",      "名义区（sim 可行安全域）"),
                        ("full",         "全域（课程学习最外圈）")]:
        ps = final["release_pose_set"][mode]
        print(f"\n【{label}】")
        p  = ps["position"]
        o  = ps["orientation"]
        ph = ps["physics"]
        print(f"  X    : {p['X_m']} m (固定)")
        y_rng_mm = p.get('Y_range_mm')
        print(f"  Y    : {format_range(y_rng_mm, 'mm', 1)}"
              f" = {format_range(p.get('Y_range_m'), 'm', 4) if p.get('Y_range_m') else 'None'}")
        print(f"  Z    : {format_range(p.get('Z_range_m'), 'm', 4)} (随 Pitch)")
        print(f"  Roll : {o['Roll_deg']}° (固定)")
        print(f"  Pitch: {format_range(o.get('Pitch_range_deg'), '°', 1)}")
        print(f"  Yaw  : {format_range(o.get('Yaw_range_deg'), '°', 1)}")
        print(f"  Mass : {format_range(ph.get('mass_range_kg'), 'kg', 2)}")
        print(f"  μ_s  : {format_range(ph.get('mu_roller_s_range'), '', 3)}")

    print()
    print("═" * 72)
    print(f"✓ 完整结果已保存: {OUTPUT_JSON}")

    # 6. 生成 markdown 报告（可选）
    write_markdown(final)
    print(f"✓ 报告已保存: {OUTPUT_MD}")
    print("═" * 72)


def write_markdown(final):
    """生成人类可读的 markdown 报告。"""
    lines = []
    lines.append(f"# Safe Release Pose Set")
    lines.append(f"\nGenerated: {final['meta']['generated_at']}")
    lines.append(f"\n- Rate threshold: {final['meta']['rate_threshold']}")
    lines.append(f"- Conservative shrink: {final['meta']['conservative_shrink']}")

    lines.append("\n## Notes")
    for note in final['meta']['notes']:
        lines.append(f"- {note}")

    lines.append("\n## Per-dimension Analysis")
    for name, a in final['analysis_per_dim'].items():
        if a is None:
            continue
        lines.append(f"\n### {name} ({a['unit']})")
        lines.append(f"- Points scanned: {a['points']}")
        lines.append(f"- Full scan range: {format_range(a['full_range'], a['unit'])}")
        lines.append(f"- Naive safe (from JSON): {format_range(a['naive_range'], a['unit'])}")
        lines.append(f"- Robust contiguous: {format_range(a['robust_range'], a['unit'])}")
        lines.append(f"- Conservative (center 60%): {format_range(a['conservative'], a['unit'])}")
        if "total_rail_lock" in a:
            lines.append(f"- Failure breakdown: success={a['total_success']}, "
                         f"rail_lock={a['total_rail_lock']}, stuck={a['total_stuck']}")

    lines.append("\n## Final Release Pose Sets")
    for mode, label in [("sim2real",     "★ Sim2Real Recommended (real deployment)"),
                        ("conservative", "Conservative (sim training core)"),
                        ("nominal",      "Nominal (feasible region)"),
                        ("full",         "Full (curriculum outermost)")]:
        ps = final['release_pose_set'][mode]
        p, o, ph = ps['position'], ps['orientation'], ps['physics']
        lines.append(f"\n### {label}")
        lines.append(f"| Dim | Range | Unit |")
        lines.append(f"|---|---|---|")
        lines.append(f"| X | {p['X_m']} | m (fixed) |")
        lines.append(f"| Y | {format_range(p.get('Y_range_mm'), '', 1)} | mm |")
        lines.append(f"| Z | {format_range(p.get('Z_range_m'), '', 4)} | m (fn of pitch) |")
        lines.append(f"| Roll | {o['Roll_deg']} | ° (fixed) |")
        lines.append(f"| Pitch | {format_range(o.get('Pitch_range_deg'), '', 1)} | ° |")
        lines.append(f"| Yaw | {format_range(o.get('Yaw_range_deg'), '', 1)} | ° |")
        lines.append(f"| Mass | {format_range(ph.get('mass_range_kg'), '', 2)} | kg |")
        lines.append(f"| μ_roller_s | {format_range(ph.get('mu_roller_s_range'), '', 3)} | - |")

    with open(OUTPUT_MD, "w") as f:
        f.write("\n".join(lines))


if __name__ == "__main__":
    main()