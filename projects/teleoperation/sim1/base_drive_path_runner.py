#!/usr/bin/env python3
"""
Run timed base-drive motions through /sim1/base_diff_cmd.

Local documentation referenced:
  - https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_generic_publisher_subscriber.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.md

Scene/control references in this workspace:
  - projects/teleoperation/sim1/base_control/script_node.py
  - projects/teleoperation/sim1/base_control/recording_notes.md
  - projects/teleoperation/sim1/graph/ALL_Graph.md
  - projects/teleoperation/sim1/graph/scene_structure.md
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


BASE_CMD_TOPIC = "/sim1/base_diff_cmd"


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


class BaseDrivePathRunner(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("base_drive_path_runner")
        self.args = args
        self.publisher = self.create_publisher(Twist, args.topic, 10)

    @staticmethod
    def make_twist(vx: float = 0.0, wz: float = 0.0) -> Twist:
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        return msg

    def publish_cmd(self, vx: float = 0.0, wz: float = 0.0) -> None:
        self.publisher.publish(self.make_twist(vx, wz))

    def run_for(self, vx: float, duration: float) -> None:
        if duration < 0.0:
            raise ValueError("duration must be >= 0")
        period = 1.0 / self.args.rate
        end_time = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_cmd(vx=vx, wz=0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)
        self.stop()

    def stop(self, frames: int | None = None) -> None:
        count = self.args.stop_frames if frames is None else frames
        period = 1.0 / self.args.rate
        for _ in range(max(count, 1)):
            if not rclpy.ok():
                break
            self.publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def turn_arc_90(self, wz: float, wait: float) -> None:
        self.publish_cmd(0.0, wz)
        rclpy.spin_once(self, timeout_sec=0.0)
        end_time = time.monotonic() + max(wait, 0.0)
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(self, timeout_sec=min(0.05, max(end_time - time.monotonic(), 0.0)))
        self.stop()

    def path_demo(self) -> None:
        speed = self.args.speed
        self.run_for(speed, 1.0 / speed)
        self.turn_arc_90(abs(self.args.trigger_wz), self.args.turn_wait)
        self.run_for(speed, 2.0 / speed)
        self.turn_arc_90(-abs(self.args.trigger_wz), self.args.turn_wait)
        self.run_for(speed, 1.0 / speed)
        self.stop()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Timed R1 base-drive motions through /sim1/base_diff_cmd."
    )
    parser.add_argument(
        "command",
        choices=("forward", "backward", "stop", "left90", "right90", "path-demo"),
    )
    parser.add_argument("--topic", default=BASE_CMD_TOPIC)
    parser.add_argument("--speed", type=float, default=0.5, help="Linear speed magnitude in m/s.")
    parser.add_argument("--duration", type=float, default=None, help="Timed forward/backward duration in seconds.")
    parser.add_argument("--rate", type=float, default=20.0, help="Publish rate in Hz.")
    parser.add_argument("--stop-frames", type=int, default=10, help="Zero-Twist frames sent when stopping.")
    parser.add_argument("--trigger-wz", type=float, default=1.0, help="Angular z trigger magnitude for arc turns.")
    parser.add_argument("--wait", type=float, default=6.0, help="Wait after left90/right90 trigger.")
    parser.add_argument("--turn-wait", type=float, default=6.0, help="Wait after each path-demo turn trigger.")
    parser.add_argument("--use-odom", action="store_true", help="Reserved for future odometry-closed-loop distance control.")
    parser.add_argument("--use-tf", action="store_true", help="Reserved for future TF-closed-loop distance control.")
    parser.add_argument("--base-frame", default="base_link", help="Reserved future closed-loop base frame.")
    return parser


def validate_args(args: argparse.Namespace) -> int:
    if args.rate <= 0.0:
        print("--rate must be > 0", file=sys.stderr)
        return 2
    if args.stop_frames <= 0:
        print("--stop-frames must be > 0", file=sys.stderr)
        return 2
    if args.speed <= 0.0:
        print("--speed must be > 0", file=sys.stderr)
        return 2
    if args.command in ("forward", "backward") and args.duration is None:
        print("--duration is required for forward/backward", file=sys.stderr)
        return 2
    if args.duration is not None and args.duration < 0.0:
        print("--duration must be >= 0", file=sys.stderr)
        return 2
    if args.wait < 0.0 or args.turn_wait < 0.0:
        print("--wait/--turn-wait must be >= 0", file=sys.stderr)
        return 2
    if args.trigger_wz == 0.0:
        print("--trigger-wz must be non-zero", file=sys.stderr)
        return 2
    return 0


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    status = validate_args(args)
    if status != 0:
        return status

    ensure_ros_log_dir()
    rclpy.init(args=None)
    node = BaseDrivePathRunner(args)
    try:
        if args.command == "forward":
            node.run_for(abs(args.speed), float(args.duration))
        elif args.command == "backward":
            node.run_for(-abs(args.speed), float(args.duration))
        elif args.command == "stop":
            node.stop()
        elif args.command == "left90":
            node.turn_arc_90(abs(args.trigger_wz), args.wait)
        elif args.command == "right90":
            node.turn_arc_90(-abs(args.trigger_wz), args.wait)
        elif args.command == "path-demo":
            node.path_demo()
    except (KeyboardInterrupt, ExternalShutdownException):
        node.stop()
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
