#!/usr/bin/env python3
"""
Replay a recorded game-style OCS2 target trace.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.md

Scene/control references in this workspace:
  - /home/gtk/Trajectory/SIM1/底盘差速驱动/script_node.py
  - /home/gtk/Trajectory/SIM1/graph/ALL_Graph.md
  - /home/gtk/Trajectory/SIM1/graph/Gripper_Control_Graph.md
  - /home/gtk/Trajectory/SIM1/keyboard_ocs2_gripper_teleop.py
  - /home/gtk/Trajectory/SIM1/script_gripper_v4.2.py
  - /home/gtk/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.2.py
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import sys
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32


LEFT_TARGET_TOPIC = "/left_target/stamped"
RIGHT_TARGET_TOPIC = "/right_target/stamped"
LEFT_GRIPPER_TOPIC = "/left_gripper_controller/commands"
RIGHT_GRIPPER_TOPIC = "/right_gripper_controller/commands"
FSM_TOPIC = "/fsm_command"
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


def parse_vec(text: str, count: int) -> list[float] | None:
    text = (text or "").strip()
    if not text:
        return None
    values = [float(x) for x in text.split()]
    if len(values) != count:
        raise ValueError(f"Expected {count} values, got {len(values)} from {text!r}")
    if any(math.isnan(v) or math.isinf(v) for v in values):
        return None
    return values


def parse_float(text: str) -> float | None:
    text = (text or "").strip()
    if not text:
        return None
    value = float(text)
    if math.isnan(value) or math.isinf(value):
        return None
    return value


def load_trace(path: Path) -> list[dict[str, object]]:
    if not path.exists():
        raise FileNotFoundError(path)
    rows: list[dict[str, object]] = []
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        required = {
            "wall_time",
            "left_target_xyz",
            "right_target_xyz",
            "left_target_xyzw",
            "right_target_xyzw",
            "left_gripper_cmd",
            "right_gripper_cmd",
        }
        missing = required.difference(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path} is missing columns: {sorted(missing)}")
        has_base_cmd = {"base_cmd_vx", "base_cmd_wz"}.issubset(set(reader.fieldnames or []))
        for raw in reader:
            left_xyz = parse_vec(raw["left_target_xyz"], 3)
            right_xyz = parse_vec(raw["right_target_xyz"], 3)
            left_xyzw = parse_vec(raw["left_target_xyzw"], 4)
            right_xyzw = parse_vec(raw["right_target_xyzw"], 4)
            if left_xyz is None or right_xyz is None or left_xyzw is None or right_xyzw is None:
                continue
            row = {
                "wall_time": float(raw["wall_time"]),
                "left_xyz": left_xyz,
                "right_xyz": right_xyz,
                "left_xyzw": left_xyzw,
                "right_xyzw": right_xyzw,
                "left_gripper": parse_float(raw["left_gripper_cmd"]),
                "right_gripper": parse_float(raw["right_gripper_cmd"]),
                "base_vx": None,
                "base_wz": None,
            }
            if has_base_cmd:
                row["base_vx"] = parse_float(raw.get("base_cmd_vx", ""))
                row["base_wz"] = parse_float(raw.get("base_cmd_wz", ""))
            rows.append(row)
    if not rows:
        raise ValueError(f"No usable target rows in {path}")
    return rows


class Ocs2TargetTraceReplay(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("ocs2_target_trace_replay")
        self.args = args
        self.left_target_pub = self.create_publisher(PoseStamped, args.left_target_topic, 10)
        self.right_target_pub = self.create_publisher(PoseStamped, args.right_target_topic, 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, args.left_gripper_topic, 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, args.right_gripper_topic, 10)
        self.fsm_pub = self.create_publisher(Int32, args.fsm_topic, 10)
        self.base_cmd_pub = self.create_publisher(Twist, args.base_cmd_topic, 10)
        self.last_left_gripper: float | None = None
        self.last_right_gripper: float | None = None

    def publish_fsm_ocs2(self) -> None:
        msg = Int32()
        msg.data = 3
        self.fsm_pub.publish(msg)

    def make_pose(self, xyz: list[float], xyzw: list[float]) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.args.frame_id
        msg.pose.position.x = xyz[0]
        msg.pose.position.y = xyz[1]
        msg.pose.position.z = xyz[2]
        msg.pose.orientation.x = xyzw[0]
        msg.pose.orientation.y = xyzw[1]
        msg.pose.orientation.z = xyzw[2]
        msg.pose.orientation.w = xyzw[3]
        return msg

    def publish_row(self, row: dict[str, object]) -> None:
        left = self.make_pose(row["left_xyz"], row["left_xyzw"])
        right = self.make_pose(row["right_xyz"], row["right_xyzw"])
        self.left_target_pub.publish(left)
        self.right_target_pub.publish(right)

        left_gripper = row["left_gripper"]
        if left_gripper is not None and left_gripper != self.last_left_gripper:
            msg = Float64MultiArray()
            msg.data = [float(left_gripper)]
            self.left_gripper_pub.publish(msg)
            self.last_left_gripper = float(left_gripper)

        right_gripper = row["right_gripper"]
        if right_gripper is not None and right_gripper != self.last_right_gripper:
            msg = Float64MultiArray()
            msg.data = [float(right_gripper)]
            self.right_gripper_pub.publish(msg)
            self.last_right_gripper = float(right_gripper)

        base_vx = row.get("base_vx")
        base_wz = row.get("base_wz")
        if base_vx is not None and base_wz is not None:
            self.publish_base_cmd(float(base_vx), float(base_wz))

    def publish_base_cmd(self, vx: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = wz
        self.base_cmd_pub.publish(msg)

    def stop_base(self) -> None:
        for _ in range(max(self.args.base_stop_frames, 1)):
            if not rclpy.ok():
                break
            self.publish_base_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Replay an OCS2 target trace CSV recorded by keyboard teleop.")
    parser.add_argument("--trace", type=Path, default=Path("manual_ocs2_keyboard_trace.csv"))
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--loop", action="store_true")
    parser.add_argument("--no-activate-ocs2", action="store_true")
    parser.add_argument("--frame-id", default="base_link")
    parser.add_argument("--left-target-topic", default=LEFT_TARGET_TOPIC)
    parser.add_argument("--right-target-topic", default=RIGHT_TARGET_TOPIC)
    parser.add_argument("--left-gripper-topic", default=LEFT_GRIPPER_TOPIC)
    parser.add_argument("--right-gripper-topic", default=RIGHT_GRIPPER_TOPIC)
    parser.add_argument("--fsm-topic", default=FSM_TOPIC)
    parser.add_argument("--base-cmd-topic", default=BASE_CMD_TOPIC)
    parser.add_argument("--base-stop-frames", type=int, default=10)
    return parser


def replay_once(node: Ocs2TargetTraceReplay, rows: list[dict[str, object]]) -> None:
    first_time = float(rows[0]["wall_time"])
    wall_start = time.monotonic()
    for row in rows:
        if not rclpy.ok():
            break
        trace_dt = (float(row["wall_time"]) - first_time) / max(node.args.speed, 1e-6)
        target_time = wall_start + max(trace_dt, 0.0)
        while rclpy.ok():
            delay = target_time - time.monotonic()
            if delay <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(delay, 0.02))
        node.publish_row(row)
        rclpy.spin_once(node, timeout_sec=0.0)


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.speed <= 0.0:
        print("--speed must be > 0", file=sys.stderr)
        return 2
    if args.base_stop_frames <= 0:
        print("--base-stop-frames must be > 0", file=sys.stderr)
        return 2
    rows = load_trace(args.trace)
    print(f"trace: {args.trace}")
    print(f"usable rows: {len(rows)}")
    print(f"duration: {float(rows[-1]['wall_time']) - float(rows[0]['wall_time']):.3f}s")
    has_base_cmd = any(row.get("base_vx") is not None and row.get("base_wz") is not None for row in rows)
    print(f"base command replay: {'enabled' if has_base_cmd else 'disabled'}")

    ensure_ros_log_dir()
    rclpy.init(args=None)
    node = Ocs2TargetTraceReplay(args)
    try:
        if not args.no_activate_ocs2:
            node.publish_fsm_ocs2()
        while rclpy.ok():
            replay_once(node, rows)
            if not args.loop:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_base()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
