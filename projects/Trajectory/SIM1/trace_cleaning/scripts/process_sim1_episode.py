#!/usr/bin/env python3
"""
Process one SIM1 raw episode into cleaned data and LeRobotDataset v3.0,
optionally also generating LeRobotDataset v2.1.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/rclpy.node.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md

External schema references:
  - https://huggingface.co/docs/lerobot/lerobot-dataset-v3
  - https://huggingface.co/docs/lerobot/main/porting_datasets_v3
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
ROOT = SCRIPT_DIR.parents[1]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Raw SIM1 episode -> cleaned -> LeRobot v3.0, optionally v2.1."
    )
    parser.add_argument("raw_trace", type=Path)
    parser.add_argument("--task", default="OCS2 keyboard teleoperation trace")
    parser.add_argument("--fps", type=int, default=20)
    parser.add_argument("--allow-missing-effort", action="store_true")
    parser.add_argument("--missing-gripper-closed", type=float, default=None)
    parser.add_argument("--pre-roll", type=float, default=0.25)
    parser.add_argument("--post-roll", type=float, default=1.0)
    parser.add_argument("--max-output-gap", type=float, default=0.10)
    parser.add_argument("--min-idle-gap", type=float, default=1.25)
    parser.add_argument(
        "--with-v21",
        action="store_true",
        help="Also generate a LeRobot v2.1 dataset. Disabled by default.",
    )
    return parser.parse_args()


def run(cmd: list[str]) -> None:
    print("+ " + " ".join(cmd))
    subprocess.run(cmd, check=True, cwd=str(ROOT))


def main() -> int:
    args = parse_args()
    raw_trace = args.raw_trace
    if not raw_trace.exists():
        raise FileNotFoundError(raw_trace)

    run_id = raw_trace.with_suffix("").name
    cleaned_trace = ROOT / "trace_data/cleaned" / f"{run_id}_cleaned.csv"
    cleaned_camera = cleaned_trace.with_suffix("") / "camera"
    v30_output = ROOT / "trace_data/lerobot_v30" / f"{run_id}_cleaned"
    v21_output = ROOT / "trace_data/lerobot_v21" / f"{run_id}_cleaned"

    run(
        [
            sys.executable,
            str(SCRIPT_DIR / "clean_sim1_episode.py"),
            str(raw_trace),
            "--output",
            str(cleaned_trace),
            "--pre-roll",
            str(args.pre_roll),
            "--post-roll",
            str(args.post_roll),
            "--max-output-gap",
            str(args.max_output_gap),
            "--min-idle-gap",
            str(args.min_idle_gap),
            "--rgb-fps",
            str(args.fps),
        ]
    )

    common = [
        "--task",
        args.task,
        "--fps",
        str(args.fps),
        "--camera-dir",
        str(cleaned_camera),
    ]
    if args.allow_missing_effort:
        common.append("--allow-missing-effort")
    if args.missing_gripper_closed is not None:
        common.extend(["--missing-gripper-closed", str(args.missing_gripper_closed)])

    run(
        [
            sys.executable,
            str(SCRIPT_DIR / "convert_ocs2_trace_to_lerobot_v30.py"),
            str(cleaned_trace),
            "--output",
            str(v30_output),
            *common,
        ]
    )
    if args.with_v21:
        run(
            [
                sys.executable,
                str(SCRIPT_DIR / "convert_ocs2_trace_to_lerobot_v21.py"),
                str(cleaned_trace),
                "--output",
                str(v21_output),
                *common,
            ]
        )

    print(f"raw: {raw_trace}")
    print(f"cleaned: {cleaned_trace}")
    print(f"lerobot_v30: {v30_output}")
    if args.with_v21:
        print(f"lerobot_v21: {v21_output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
