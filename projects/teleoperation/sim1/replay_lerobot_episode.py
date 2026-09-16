#!/usr/bin/env python3
"""
Replay a converted LeRobot v3 dual-arm episode into Isaac Sim over ROS 2.

Local documentation referenced:
  - https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.md

Scene/control references in this workspace:
  - projects/teleoperation/sim1/graph/ALL_Graph.md
  - projects/teleoperation/sim1/graph/Gripper_Control_Graph.md
  - projects/teleoperation/sim1/script_gripper_v4.2.py
  - projects/teleoperation/converted_lerobot/local/place_tray_middle/export_rm_dual_episode.py
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


DEFAULT_TRAJECTORY_DIR = Path(os.environ.get("TRAJECTORY_DIR", str(Path(__file__).resolve().parents[1])))
DEFAULT_EPISODE = (
    DEFAULT_TRAJECTORY_DIR
    / "converted_lerobot/local/place_tray_middle/episode_000_rm_dual_joint_traj.npz"
)

LEFT_GRIPPER_TOPIC = "/left_gripper_controller/commands"
RIGHT_GRIPPER_TOPIC = "/right_gripper_controller/commands"


def ensure_ros_log_dir() -> None:
    if os.environ.get("ROS_LOG_DIR"):
        return
    default_log_dir = Path.home() / ".ros/log"
    try:
        default_log_dir.mkdir(parents=True, exist_ok=True)
        probe = default_log_dir / ".codex_write_probe"
        probe.write_text("", encoding="utf-8")
        probe.unlink()
    except OSError:
        fallback = Path("/tmp") / f"ros_logs_{os.environ.get('USER', 'user')}"
        fallback.mkdir(parents=True, exist_ok=True)
        os.environ["ROS_LOG_DIR"] = str(fallback)


class LeRobotEpisodeReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("lerobot_episode_replay")
        self.args = args
        self.arm_pub = self.create_publisher(JointState, args.arm_topic, 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, args.left_gripper_topic, 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, args.right_gripper_topic, 10)
        self.csv_writer = None
        self.csv_file = None

        if args.record_csv:
            args.record_csv.parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = args.record_csv.open("w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                [
                    "wall_time",
                    "episode_time",
                    "frame",
                    "joint_names",
                    "positions",
                    "left_gripper_cmd",
                    "right_gripper_cmd",
                    "left_gripper_raw",
                    "right_gripper_raw",
                ]
            )

    def close(self) -> None:
        if self.csv_file:
            self.csv_file.close()

    def publish_arm(self, names: list[str], positions: np.ndarray) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = names
        msg.position = [float(v) for v in positions]
        self.arm_pub.publish(msg)

    def publish_grippers(self, raw: np.ndarray) -> tuple[float, float]:
        if self.args.no_gripper:
            return math.nan, math.nan

        if self.args.gripper_mode == "raw":
            left_cmd = float(raw[0])
            right_cmd = float(raw[1])
        else:
            left_cmd = 1.0 if float(raw[0]) >= self.args.gripper_open_threshold else 0.0
            right_cmd = 1.0 if float(raw[1]) >= self.args.gripper_open_threshold else 0.0

        left_msg = Float64MultiArray()
        right_msg = Float64MultiArray()
        left_msg.data = [left_cmd]
        right_msg.data = [right_cmd]
        self.left_gripper_pub.publish(left_msg)
        self.right_gripper_pub.publish(right_msg)
        return left_cmd, right_cmd

    def write_csv(
        self,
        episode_time: float,
        frame: int,
        names: list[str],
        positions: np.ndarray,
        gripper_raw: np.ndarray,
        gripper_cmd: tuple[float, float],
    ) -> None:
        if not self.csv_writer:
            return
        self.csv_writer.writerow(
            [
                f"{time.time():.9f}",
                f"{episode_time:.9f}",
                frame,
                " ".join(names),
                " ".join(f"{float(v):.9g}" for v in positions),
                f"{gripper_cmd[0]:.9g}",
                f"{gripper_cmd[1]:.9g}",
                f"{float(gripper_raw[0]):.9g}",
                f"{float(gripper_raw[1]):.9g}",
            ]
        )


def load_episode(path: Path) -> tuple[np.ndarray, np.ndarray, np.ndarray, list[str]]:
    if not path.exists():
        raise FileNotFoundError(path)

    data = np.load(path, allow_pickle=True)
    required = {"t", "q", "gripper", "joint_names"}
    missing = required.difference(data.files)
    if missing:
        raise ValueError(f"{path} is missing fields: {sorted(missing)}")

    t = np.asarray(data["t"], dtype=np.float64)
    q = np.asarray(data["q"], dtype=np.float64)
    gripper = np.asarray(data["gripper"], dtype=np.float64)
    names = [str(x) for x in data["joint_names"].tolist()]

    if t.ndim != 1 or q.ndim != 2 or gripper.ndim != 2:
        raise ValueError("Expected t=(N,), q=(N,J), gripper=(N,2)")
    if len(t) != len(q) or len(t) != len(gripper):
        raise ValueError("t, q, and gripper must have the same frame count")
    if q.shape[1] != len(names):
        raise ValueError("q width does not match joint_names length")
    if gripper.shape[1] != 2:
        raise ValueError("gripper must have two columns: left, right")

    return t, q, gripper, names


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Replay episode_000_rm_dual_joint_traj.npz into Isaac Sim."
    )
    parser.add_argument("--episode", type=Path, default=DEFAULT_EPISODE)
    parser.add_argument("--arm-topic", default="/arm_joint_cmd")
    parser.add_argument("--left-gripper-topic", default=LEFT_GRIPPER_TOPIC)
    parser.add_argument("--right-gripper-topic", default=RIGHT_GRIPPER_TOPIC)
    parser.add_argument("--speed", type=float, default=1.0, help="Playback speed multiplier.")
    parser.add_argument("--start", type=float, default=0.0, help="Start episode time in seconds.")
    parser.add_argument("--duration", type=float, default=None, help="Optional playback duration in seconds.")
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--no-gripper", action="store_true")
    parser.add_argument(
        "--gripper-mode",
        choices=("binary", "raw"),
        default="binary",
        help="binary sends 0/1 commands expected by the current PGIA script nodes.",
    )
    parser.add_argument("--gripper-open-threshold", type=float, default=0.5)
    parser.add_argument("--record-csv", type=Path, default=None)
    return parser


def replay_once(node: LeRobotEpisodeReplay, t: np.ndarray, q: np.ndarray, gripper: np.ndarray, names: list[str]) -> None:
    args = node.args
    start_index = int(np.searchsorted(t, args.start, side="left"))
    if start_index >= len(t):
        raise ValueError(f"--start {args.start} is beyond episode duration {t[-1]:.3f}s")

    if args.duration is None:
        end_time = float(t[-1])
    else:
        end_time = min(float(t[-1]), args.start + args.duration)
    end_index = int(np.searchsorted(t, end_time, side="right"))

    playback_t0 = time.monotonic()
    episode_t0 = float(t[start_index])

    for i in range(start_index, end_index):
        if not rclpy.ok():
            break
        episode_time = float(t[i])
        target_wall = playback_t0 + (episode_time - episode_t0) / max(args.speed, 1e-6)
        while not args.dry_run and rclpy.ok():
            delay = target_wall - time.monotonic()
            if delay <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(delay, 0.02))

        gripper_cmd = (math.nan, math.nan)
        if not args.dry_run:
            node.publish_arm(names, q[i])
            gripper_cmd = node.publish_grippers(gripper[i])
            rclpy.spin_once(node, timeout_sec=0.0)
        node.write_csv(episode_time, i, names, q[i], gripper[i], gripper_cmd)

    if not args.dry_run and not args.no_gripper:
        node.publish_grippers(gripper[end_index - 1])


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.speed <= 0.0:
        print("--speed must be > 0", file=sys.stderr)
        return 2

    t, q, gripper, names = load_episode(args.episode)
    print(f"episode: {args.episode}")
    print(f"frames: {len(t)} duration: {float(t[-1]):.3f}s joints: {len(names)}")
    print(f"arm topic: {args.arm_topic}")
    print(f"gripper topics: {args.left_gripper_topic}, {args.right_gripper_topic}")

    ensure_ros_log_dir()
    rclpy.init(args=None)
    node = LeRobotEpisodeReplay(args)
    try:
        while rclpy.ok():
            replay_once(node, t, q, gripper, names)
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
