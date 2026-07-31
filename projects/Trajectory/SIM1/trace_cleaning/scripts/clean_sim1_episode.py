#!/usr/bin/env python3
"""
Clean a SIM1 raw episode CSV and its synchronized camera data.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/rclpy.node.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md

The trace cleaner removes idle observation intervals. This wrapper applies the
same keep intervals to camera_timestamps.csv, RGB mp4 files, and depth npy
frames so cleaned data stays timestamp-alignable without frame-index matching.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import sys
from bisect import bisect_right
from pathlib import Path

import cv2

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import clean_ocs2_trace as trace_cleaner


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Clean SIM1 trace CSV plus camera data.")
    parser.add_argument("source", type=Path, help="Raw trace CSV.")
    parser.add_argument("--output", type=Path, default=None, help="Cleaned trace CSV.")
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--camera-dir", type=Path, default=None)
    parser.add_argument("--pre-roll", type=float, default=0.25)
    parser.add_argument("--post-roll", type=float, default=1.0)
    parser.add_argument("--max-output-gap", type=float, default=0.10)
    parser.add_argument("--min-idle-gap", type=float, default=1.25)
    parser.add_argument("--rgb-fps", type=float, default=20.0)
    return parser.parse_args()


def in_intervals(t: float, intervals: list[tuple[float, float]]) -> bool:
    return any(start <= t <= end for start, end in intervals)


class TimeMapper:
    def __init__(self, source_rows: list[dict[str, str]], cleaned_rows: list[dict[str, str]]) -> None:
        self.source_wall = [float(row["wall_time"]) for row in source_rows]
        self.source_ros = [float(row["ros_time_sec"]) for row in source_rows]
        self.cleaned_wall = [float(row["wall_time"]) for row in cleaned_rows]
        self.cleaned_ros = [float(row["ros_time_sec"]) for row in cleaned_rows]

    def map_wall(self, t: float) -> float:
        return self._map(t, self.source_wall, self.cleaned_wall)

    def map_ros_from_wall(self, t: float) -> float:
        return self._map(t, self.source_wall, self.cleaned_ros)

    @staticmethod
    def _map(t: float, source: list[float], target: list[float]) -> float:
        if not source:
            return t
        if t <= source[0]:
            return target[0] + (t - source[0])
        if t >= source[-1]:
            return target[-1] + (t - source[-1])
        idx = bisect_right(source, t) - 1
        idx = max(0, min(idx, len(source) - 2))
        span = source[idx + 1] - source[idx]
        alpha = 0.0 if span <= 1e-9 else (t - source[idx]) / span
        return target[idx] + alpha * (target[idx + 1] - target[idx])


def resolve_camera_dir(source: Path, explicit: Path | None) -> Path | None:
    if explicit is not None:
        return explicit if explicit.exists() else None
    sibling = source.with_suffix("") / "camera"
    if sibling.exists():
        return sibling
    direct = source.with_suffix("")
    if (direct / "camera_timestamps.csv").exists():
        return direct
    return None


def load_camera_rows(camera_dir: Path) -> list[dict[str, str]]:
    path = camera_dir / "camera_timestamps.csv"
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def trim_rgb_video(
    source_video: Path,
    dest_video: Path,
    camera_rows: list[dict[str, str]],
    kept_source_indices: set[int],
    fps: float,
) -> int:
    cap = cv2.VideoCapture(str(source_video))
    if not cap.isOpened():
        raise RuntimeError(f"failed to open video: {source_video}")
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or int(camera_rows[0].get("width") or 640)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or int(camera_rows[0].get("height") or 480)
    dest_video.parent.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(dest_video), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        cap.release()
        raise RuntimeError(f"failed to open video writer: {dest_video}")
    written = 0
    source_index = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if source_index in kept_source_indices:
            writer.write(frame)
            written += 1
        source_index += 1
    writer.release()
    cap.release()
    return written


def clean_camera_data(
    camera_dir: Path | None,
    output_camera_dir: Path,
    intervals: list[tuple[float, float]],
    mapper: TimeMapper,
    rgb_fps: float,
) -> dict[str, object]:
    if camera_dir is None:
        return {"camera_cleaned": False, "reason": "no camera directory found"}

    rows = load_camera_rows(camera_dir)
    if not rows:
        return {"camera_cleaned": False, "reason": "no camera_timestamps.csv found"}

    output_camera_dir.mkdir(parents=True, exist_ok=True)
    kept_rows = []
    per_camera: dict[str, list[tuple[int, dict[str, str]]]] = {}
    for row in rows:
        wall = float(row["wall_time_sec"])
        if not in_intervals(wall, intervals):
            continue
        camera_name = row["camera_name"]
        source_index = int(row["frame_index"])
        new_row = dict(row)
        new_row["frame_index"] = str(len(per_camera.setdefault(camera_name, [])))
        new_row["wall_time_sec"] = f"{mapper.map_wall(wall):.9f}"
        new_row["ros_time_sec"] = f"{mapper.map_ros_from_wall(wall):.9f}"
        per_camera[camera_name].append((source_index, new_row))
        kept_rows.append(new_row)

    for camera_name, indexed_rows in per_camera.items():
        if not indexed_rows:
            continue
        stream_type = indexed_rows[0][1]["stream_type"]
        if stream_type == "rgb":
            rel_path = indexed_rows[0][1]["relative_path"]
            source_video = camera_dir / rel_path
            dest_rel = Path("videos") / f"{camera_name}.mp4"
            dest_video = output_camera_dir / dest_rel
            trim_rgb_video(source_video, dest_video, [row for _, row in indexed_rows], {i for i, _ in indexed_rows}, rgb_fps)
            for _, row in indexed_rows:
                row["relative_path"] = str(dest_rel)
        elif stream_type == "depth":
            for new_index, (source_index, row) in enumerate(indexed_rows):
                source_rel = row["relative_path"]
                if not source_rel:
                    continue
                source_depth = camera_dir / source_rel
                dest_rel = Path("depth") / camera_name / f"{new_index:06d}.npy"
                dest_depth = output_camera_dir / dest_rel
                dest_depth.parent.mkdir(parents=True, exist_ok=True)
                if source_depth.exists():
                    shutil.copy2(source_depth, dest_depth)
                    row["relative_path"] = str(dest_rel)

    fieldnames = list(rows[0])
    with (output_camera_dir / "camera_timestamps.csv").open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(kept_rows)

    summary = {
        "source_camera_dir": str(camera_dir),
        "output_dir": str(output_camera_dir),
        "camera_cleaned": True,
        "streams": {},
    }
    for camera_name, indexed_rows in sorted(per_camera.items()):
        summary["streams"][camera_name] = {
            "frame_count": len(indexed_rows),
            "type": indexed_rows[0][1]["stream_type"] if indexed_rows else "",
            "topic": indexed_rows[0][1]["topic"] if indexed_rows else "",
            "output": indexed_rows[0][1]["relative_path"] if indexed_rows else "",
        }
    (output_camera_dir / "summary.json").write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    return {"camera_cleaned": True, "kept_camera_rows": len(kept_rows), "streams": summary["streams"]}


def main() -> int:
    args = parse_args()
    source = args.source
    if args.output is None:
        sim1_dir = Path(os.environ.get("SIM1_DIR", "/workspace/projects/Trajectory/SIM1"))
        args.output = sim1_dir / "trace_data/cleaned" / f"{source.stem}_cleaned.csv"
    if args.report is None:
        args.report = args.output.with_suffix(".report.md")

    cleaner_args = argparse.Namespace(
        pre_roll=args.pre_roll,
        post_roll=args.post_roll,
        max_output_gap=args.max_output_gap,
        min_idle_gap=args.min_idle_gap,
    )
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

    keep_mask, intervals = trace_cleaner.build_keep_mask(rows, cleaner_args)
    selected = [row for row, keep in zip(rows, keep_mask) if keep]
    cleaned = trace_cleaner.compact_times(selected, args.max_output_gap)
    trace_cleaner.write_csv(args.output, fieldnames, cleaned)
    trace_cleaner.write_report(args.report, source, args.output, rows, cleaned, intervals)

    camera_result = {"camera_cleaned": False, "reason": "no kept trace rows"}
    if cleaned:
        source_selected = [row for row, keep in zip(rows, keep_mask) if keep]
        mapper = TimeMapper(source_selected, cleaned)
        camera_result = clean_camera_data(
            resolve_camera_dir(source, args.camera_dir),
            args.output.with_suffix("") / "camera",
            intervals,
            mapper,
            args.rgb_fps,
        )
    with args.report.open("a", encoding="utf-8") as f:
        f.write("\n## Camera Cleaning\n\n")
        for key, value in camera_result.items():
            f.write(f"- {key}: `{value}`\n")

    print(f"source rows: {len(rows)}")
    print(f"cleaned rows: {len(cleaned)}")
    print(f"output: {args.output}")
    print(f"report: {args.report}")
    print(f"camera: {args.output.with_suffix('') / 'camera'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
