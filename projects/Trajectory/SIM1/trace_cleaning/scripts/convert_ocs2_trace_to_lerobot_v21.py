#!/usr/bin/env python3
"""
Convert cleaned SIM1 OCS2 keyboard traces to a local LeRobotDataset v2.1 layout.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/rclpy.node.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md

External schema references:
  - https://huggingface.co/docs/lerobot/main/porting_datasets_v3
  - /home/gtk/.local/lib/python3.10/site-packages/lerobot/datasets/lerobot_dataset.py
  - /home/gtk/.local/lib/python3.10/site-packages/lerobot/datasets/utils.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import pandas as pd

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from sim1_lerobot_common import (
    ARM_JOINTS,
    CAMERA_VIDEO_KEYS,
    POSE_NAMES,
    STATE_NAMES,
    build_dataset_rows,
    camera_dir_for_trace,
    camera_shape,
    copy_selected_videos,
    feature,
    load_camera_rows,
    load_trace,
    video_feature,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert a cleaned SIM1 OCS2 trace CSV to LeRobotDataset v2.1.")
    parser.add_argument("trace", type=Path, help="Cleaned OCS2 trace CSV.")
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--camera-dir", type=Path, default=None)
    parser.add_argument("--task", default="OCS2 keyboard teleoperation trace")
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--task-index", type=int, default=0)
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--allow-missing-effort", action="store_true")
    parser.add_argument("--missing-gripper-closed", type=float, default=None)
    return parser.parse_args()


def default_output(trace: Path) -> Path:
    sim1_dir = Path(os.environ.get("SIM1_DIR", "/workspace/projects/Trajectory/SIM1"))
    return sim1_dir / "trace_data/lerobot_v21" / trace.with_suffix("").name


def jsonline(obj: dict[str, object]) -> str:
    return json.dumps(obj, ensure_ascii=False) + "\n"


def write_jsonl(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("".join(jsonline(row) for row in rows), encoding="utf-8")


def write_dataset(
    data: pd.DataFrame,
    stats: dict[str, object],
    args: argparse.Namespace,
    camera_dir: Path | None,
    camera_keys_present: list[str],
) -> None:
    root = args.output
    data_dir = root / "data/chunk-000"
    meta_dir = root / "meta"
    data_dir.mkdir(parents=True, exist_ok=True)
    meta_dir.mkdir(parents=True, exist_ok=True)

    parquet_data = data[[col for col in data.columns if not col.startswith("observation.images.")]]
    parquet_data.to_parquet(data_dir / f"episode_{args.episode_index:06d}.parquet", index=False)

    length = int(len(data))
    features = {
        "observation.state": feature("float32", [16], STATE_NAMES, args.fps),
        "observation.state.pose": feature("float32", [16], POSE_NAMES, args.fps),
        "observation.effort": feature("float32", [14], ARM_JOINTS, args.fps),
        "action": feature("float32", [16], STATE_NAMES, args.fps),
        "action.pose": feature("float32", [16], POSE_NAMES, args.fps),
        "timestamp": feature("float32", [1], None, args.fps),
        "frame_index": feature("int64", [1], None, args.fps),
        "episode_index": feature("int64", [1], None, args.fps),
        "index": feature("int64", [1], None, args.fps),
        "task_index": feature("int64", [1], None, args.fps),
    }
    for camera_name in camera_keys_present:
        features[CAMERA_VIDEO_KEYS[camera_name]] = video_feature(camera_shape(camera_dir, camera_name), args.fps)

    info = {
        "codebase_version": "v2.1",
        "robot_type": "rm_dual",
        "total_episodes": 1,
        "total_frames": length,
        "total_tasks": 1,
        "total_chunks": 1,
        "chunks_size": 1000,
        "fps": args.fps,
        "splits": {"train": "0:1"},
        "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
        "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4"
        if camera_keys_present
        else None,
        "features": features,
    }
    (meta_dir / "info.json").write_text(json.dumps(info, indent=4) + "\n", encoding="utf-8")
    write_jsonl(meta_dir / "tasks.jsonl", [{"task_index": args.task_index, "task": args.task}])
    write_jsonl(
        meta_dir / "episodes.jsonl",
        [{"episode_index": args.episode_index, "tasks": [args.task], "length": length}],
    )
    episode_stats = {"episode_index": args.episode_index, "stats": stats}
    write_jsonl(meta_dir / "episodes_stats.jsonl", [episode_stats])
    (meta_dir / "stats.json").write_text(json.dumps(stats, indent=4) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    if not args.trace.exists():
        raise FileNotFoundError(args.trace)
    if args.fps <= 0:
        raise ValueError("--fps must be > 0")
    if args.output is None:
        args.output = default_output(args.trace)

    camera_dir = camera_dir_for_trace(args.trace, args.camera_dir)
    camera_rows = load_camera_rows(camera_dir)
    rows = load_trace(args.trace)
    data, stats, camera_indices = build_dataset_rows(
        rows,
        missing_gripper_closed=args.missing_gripper_closed,
        allow_missing_effort=args.allow_missing_effort,
        camera_rows_by_name=camera_rows,
    )
    copy_selected_videos(camera_dir, args.output, camera_rows, camera_indices, v30=False)
    write_dataset(data, stats, args, camera_dir, sorted(camera_indices))

    print(f"frames: {len(data)}")
    print(f"dataset: {args.output}")
    print(f"data: {args.output / 'data/chunk-000' / f'episode_{args.episode_index:06d}.parquet'}")
    print(f"meta: {args.output / 'meta/info.json'}")
    if camera_indices:
        print(f"videos: {args.output / 'videos'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
