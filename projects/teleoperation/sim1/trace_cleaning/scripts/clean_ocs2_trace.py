#!/usr/bin/env python3
"""
Clean game-style OCS2 keyboard trace CSVs for replay.

Local documentation referenced:
  - https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md

The replay script schedules rows from wall_time, so this cleaner removes long
idle observation gaps and compacts wall_time/ros_time_sec accordingly.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
from pathlib import Path


CONTROL_EVENTS = {
    "w",
    "s",
    "a",
    "d",
    "r",
    "f",
    "up",
    "down",
    "left",
    "right",
    "u",
    "o",
    "i",
    "k",
    "j",
    "l",
    "toggle_gripper",
    "left_close",
    "left_open",
    "right_close",
    "right_open",
    "both_close",
    "both_open",
    "base_forward",
    "base_backward",
    "base_left_arc_90",
    "base_right_arc_90",
    "base_stop",
    "base_blocked_multi_arrow",
}
DROP_EVENTS = {"quit", "keyboard_interrupt", "reset_after_task", "reset_and_continue"}
STATE_COLUMNS = [
    "left_target_xyz",
    "right_target_xyz",
    "left_target_xyzw",
    "right_target_xyzw",
    "left_gripper_cmd",
    "right_gripper_cmd",
    "base_cmd_vx",
    "base_cmd_wz",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Remove long no-action gaps from OCS2 keyboard traces.")
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--pre-roll", type=float, default=0.25)
    parser.add_argument("--post-roll", type=float, default=1.0)
    parser.add_argument("--max-output-gap", type=float, default=0.10)
    parser.add_argument("--min-idle-gap", type=float, default=1.25)
    return parser.parse_args()


def row_time(row: dict[str, str]) -> float:
    return float(row["wall_time"])


def same_value(a: str, b: str) -> bool:
    return (a or "").strip() == (b or "").strip()


def state_changed(prev: dict[str, str] | None, row: dict[str, str]) -> bool:
    if prev is None:
        return False
    if row.get("event") in DROP_EVENTS:
        return False
    return any(not same_value(prev.get(col, ""), row.get(col, "")) for col in STATE_COLUMNS)


def is_action(prev: dict[str, str] | None, row: dict[str, str]) -> bool:
    event = row.get("event", "")
    if event in CONTROL_EVENTS:
        return True
    return state_changed(prev, row) and event not in {"tick", "P", "p", "1", "2", "3"}


def merge_intervals(intervals: list[tuple[float, float]], max_gap: float) -> list[tuple[float, float]]:
    if not intervals:
        return []
    intervals.sort()
    merged = [intervals[0]]
    for start, end in intervals[1:]:
        last_start, last_end = merged[-1]
        if start - last_end <= max_gap:
            merged[-1] = (last_start, max(last_end, end))
        else:
            merged.append((start, end))
    return merged


def build_keep_mask(rows: list[dict[str, str]], args: argparse.Namespace) -> tuple[list[bool], list[tuple[float, float]]]:
    intervals = []
    prev = None
    first_t = row_time(rows[0])
    last_t = row_time(rows[-1])
    for row in rows:
        if is_action(prev, row):
            t = row_time(row)
            intervals.append((max(first_t, t - args.pre_roll), min(last_t, t + args.post_roll)))
        prev = row

    intervals = merge_intervals(intervals, args.min_idle_gap)
    keep = []
    interval_i = 0
    for row in rows:
        if row.get("event") in DROP_EVENTS:
            keep.append(False)
            continue
        t = row_time(row)
        while interval_i < len(intervals) and t > intervals[interval_i][1]:
            interval_i += 1
        keep.append(interval_i < len(intervals) and intervals[interval_i][0] <= t <= intervals[interval_i][1])
    return keep, intervals


def compact_times(rows: list[dict[str, str]], max_gap: float) -> list[dict[str, str]]:
    if not rows:
        return []
    out = []
    source_wall0 = float(rows[0]["wall_time"])
    source_ros0 = float(rows[0]["ros_time_sec"])
    new_wall = source_wall0
    new_ros = source_ros0
    prev_wall = source_wall0
    prev_ros = source_ros0

    for index, row in enumerate(rows):
        row = dict(row)
        if index > 0:
            raw_wall_dt = max(float(row["wall_time"]) - prev_wall, 0.0)
            raw_ros_dt = max(float(row["ros_time_sec"]) - prev_ros, 0.0)
            new_wall += min(raw_wall_dt, max_gap)
            new_ros += min(raw_ros_dt, max_gap)
            prev_wall = float(row["wall_time"])
            prev_ros = float(row["ros_time_sec"])
        row["wall_time"] = f"{new_wall:.9f}"
        row["ros_time_sec"] = f"{new_ros:.9f}"
        out.append(row)
    return out


def write_csv(path: Path, fieldnames: list[str], rows: list[dict[str, str]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def write_report(
    path: Path,
    source: Path,
    output: Path,
    rows: list[dict[str, str]],
    kept: list[dict[str, str]],
    intervals: list[tuple[float, float]],
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    original_duration = row_time(rows[-1]) - row_time(rows[0])
    cleaned_duration = row_time(kept[-1]) - row_time(kept[0]) if len(kept) >= 2 else 0.0
    event_counts: dict[str, int] = {}
    for row in kept:
        event_counts[row.get("event", "")] = event_counts.get(row.get("event", ""), 0) + 1

    lines = [
        "# OCS2 Trace Cleaning Report",
        "",
        f"- source: `{source}`",
        f"- output: `{output}`",
        f"- original_rows: `{len(rows)}`",
        f"- cleaned_rows: `{len(kept)}`",
        f"- removed_rows: `{len(rows) - len(kept)}`",
        f"- original_duration_sec: `{original_duration:.3f}`",
        f"- cleaned_duration_sec: `{cleaned_duration:.3f}`",
        f"- kept_intervals: `{len(intervals)}`",
        "",
        "## Kept Event Counts",
        "",
        "| Event | Rows |",
        "| --- | ---: |",
    ]
    for event, count in sorted(event_counts.items()):
        lines.append(f"| `{event}` | {count} |")

    lines.extend(["", "## Kept Source Intervals", "", "| Start Offset | End Offset | Duration |", "| ---: | ---: | ---: |"])
    first_t = row_time(rows[0])
    for start, end in intervals:
        lines.append(f"| {start - first_t:.3f} | {end - first_t:.3f} | {end - start:.3f} |")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    source = args.source
    if args.output is None:
        sim1_dir = Path(os.environ.get("SIM1_DIR", str(Path(__file__).resolve().parents[2])))
        args.output = sim1_dir / "trace_data/cleaned" / f"{source.stem}_cleaned.csv"
    if args.report is None:
        args.report = args.output.with_suffix(".report.md")

    with source.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{source} has no CSV header")
        rows = list(reader)
        fieldnames = list(reader.fieldnames)

    if not rows:
        raise ValueError(f"{source} contains no data rows")
    if any(math.isnan(float(row["wall_time"])) for row in rows):
        raise ValueError("wall_time contains NaN")

    keep_mask, intervals = build_keep_mask(rows, args)
    selected = [row for row, keep in zip(rows, keep_mask) if keep]
    cleaned = compact_times(selected, args.max_output_gap)
    write_csv(args.output, fieldnames, cleaned)
    write_report(args.report, source, args.output, rows, cleaned, intervals)
    print(f"source rows: {len(rows)}")
    print(f"cleaned rows: {len(cleaned)}")
    print(f"output: {args.output}")
    print(f"report: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
