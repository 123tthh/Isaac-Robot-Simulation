#!/usr/bin/env python3
"""Capture the current 14-DOF dual-arm joint pose for later use as OCS2 home.

References:
- /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
- /home/gtk/Trajectory/SIM1/keyboard_ocs2_gripper_teleop.py
- /home/gtk/ros2_ws/src/robot/config/ros2_control/ocs2_controllers.yaml
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import shutil
import sys
import time
from pathlib import Path
from typing import Dict, Iterable, List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


ARM_JOINTS = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
]


def ensure_ros_log_dir() -> None:
    ros_home = Path(os.environ.get("ROS_HOME", str(Path.cwd() / ".ros")))
    log_dir = ros_home / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_HOME", str(ros_home))
    os.environ.setdefault("ROS_LOG_DIR", str(log_dir))


def finite_or_none(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    if not math.isfinite(value):
        return None
    return float(value)


def format_home_yaml(values: Iterable[float], name: str = "home_1") -> str:
    formatted = [f"{value:.9g}" for value in values]
    return (
        f"{name}: [{', '.join(formatted[:7])},\n"
        f"         {', '.join(formatted[7:])}]"
    )


class ArmPoseCaptureNode(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("sim1_arm_initial_pose_capture")
        self.args = args
        self.primary_positions: Dict[str, float] = {}
        self.fallback_positions: Dict[str, float] = {}
        self.primary_stamp_wall: Optional[float] = None
        self.fallback_stamp_wall: Optional[float] = None

        self.create_subscription(
            JointState,
            args.joint_states_topic,
            self._primary_joint_state_cb,
            20,
        )
        self.create_subscription(
            JointState,
            args.isaac_joint_states_topic,
            self._fallback_joint_state_cb,
            20,
        )

    def _update_positions(self, msg: JointState, target: Dict[str, float]) -> None:
        for name, position in zip(msg.name, msg.position):
            if name in ARM_JOINTS and math.isfinite(float(position)):
                target[name] = float(position)

    def _primary_joint_state_cb(self, msg: JointState) -> None:
        self._update_positions(msg, self.primary_positions)
        self.primary_stamp_wall = time.time()

    def _fallback_joint_state_cb(self, msg: JointState) -> None:
        self._update_positions(msg, self.fallback_positions)
        self.fallback_stamp_wall = time.time()

    def joint_value(self, name: str) -> Optional[float]:
        value = finite_or_none(self.primary_positions.get(name))
        if value is not None:
            return value
        return finite_or_none(self.fallback_positions.get(name))

    def snapshot(self) -> Dict[str, Optional[float]]:
        return {name: self.joint_value(name) for name in ARM_JOINTS}

    def missing_joints(self) -> List[str]:
        return [name for name, value in self.snapshot().items() if value is None]


def write_latest(path: Path, latest_path: Path) -> None:
    try:
        if latest_path.exists() or latest_path.is_symlink():
            latest_path.unlink()
        latest_path.symlink_to(path.name)
    except OSError:
        shutil.copy2(path, latest_path)


def write_outputs(
    out_dir: Path,
    stamp: str,
    positions: Dict[str, Optional[float]],
    source_info: Dict[str, object],
) -> List[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    values = [positions[name] for name in ARM_JOINTS]
    if any(value is None for value in values):
        raise ValueError("cannot write OCS2 home snippet with missing arm joints")
    numeric_values = [float(value) for value in values if value is not None]

    json_path = out_dir / f"arm_initial_pose_{stamp}.json"
    csv_path = out_dir / f"arm_initial_pose_{stamp}.csv"
    md_path = out_dir / f"arm_initial_pose_{stamp}.md"

    payload = {
        "created_wall_time": time.time(),
        "joint_order": ARM_JOINTS,
        "positions_rad": {name: positions[name] for name in ARM_JOINTS},
        "ocs2_home_yaml": format_home_yaml(numeric_values),
        "source": source_info,
    }
    json_path.write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n", encoding="utf-8")

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["joint_name", "position_rad"])
        for name in ARM_JOINTS:
            writer.writerow([name, f"{float(positions[name]):.12g}"])

    md_path.write_text(
        "# SIM1 Arm Initial Pose Snapshot\n\n"
        "Joint order matches `ocs2_controllers.yaml` `home_1/home_2`.\n\n"
        "```yaml\n"
        f"{format_home_yaml(numeric_values)}\n"
        "```\n\n"
        "| joint | position_rad |\n"
        "| --- | ---: |\n"
        + "\n".join(f"| {name} | {float(positions[name]):.12g} |" for name in ARM_JOINTS)
        + "\n",
        encoding="utf-8",
    )

    for path in (json_path, csv_path, md_path):
        latest = out_dir / f"arm_initial_pose_latest{path.suffix}"
        write_latest(path, latest)

    return [json_path, csv_path, md_path]


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture current SIM1 dual-arm joint positions as an OCS2 home pose snapshot."
    )
    parser.add_argument("--joint-states-topic", default="/joint_states")
    parser.add_argument("--isaac-joint-states-topic", default="/isaac_joint_states")
    parser.add_argument("--out-dir", default="trace_data/initial_poses")
    parser.add_argument("--timeout", type=float, default=5.0)
    parser.add_argument("--spin-period", type=float, default=0.05)
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    ensure_ros_log_dir()
    rclpy.init(args=None)
    node = ArmPoseCaptureNode(args)
    try:
        deadline = time.time() + max(args.timeout, 0.1)
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=max(args.spin_period, 0.01))
            if not node.missing_joints():
                break

        missing = node.missing_joints()
        if missing:
            node.get_logger().error(
                "Missing arm joints before timeout: " + ", ".join(missing)
            )
            return 2

        positions = node.snapshot()
        stamp = time.strftime("%Y%m%d_%H%M%S")
        paths = write_outputs(
            Path(args.out_dir),
            stamp,
            positions,
            {
                "joint_states_topic": args.joint_states_topic,
                "isaac_joint_states_topic": args.isaac_joint_states_topic,
                "primary_stamp_wall": node.primary_stamp_wall,
                "fallback_stamp_wall": node.fallback_stamp_wall,
            },
        )
        values = [float(positions[name]) for name in ARM_JOINTS]
        print("Captured SIM1 arm initial pose:")
        for path in paths:
            print(f"  {path}")
        print("\nOCS2 home_1 snippet:")
        print(format_home_yaml(values))
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
