#!/usr/bin/env python3
"""
Record SIM1 base-drive commands and wheel/caster states to CSV.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_generic_publisher_subscriber.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_clock.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.md

Scene/control references in this workspace:
  - /home/gtk/Trajectory/SIM1/底盘差速驱动/script_node.py
  - /home/gtk/ros2_log/scripts/底盘差速驱动/script_node_base_arc_debug_v2.py
  - /home/gtk/Trajectory/SIM1/底盘差速驱动/SIM1_base_drive_recording_handoff.md
  - /home/gtk/Trajectory/SIM1/graph/ALL_Graph.md
  - /home/gtk/Trajectory/SIM1/graph/scene_structure.md
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
from datetime import datetime
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from nav_msgs.msg import Odometry
except ImportError:  # pragma: no cover - optional ROS interface
    Odometry = None

try:
    from tf2_msgs.msg import TFMessage
except ImportError:  # pragma: no cover - optional ROS interface
    TFMessage = None


BASE_CMD_TOPIC = "/sim1/base_diff_cmd"
BASE_DEBUG_TOPIC = "/sim1/base_drive_debug"
ISAAC_JOINT_STATES_TOPIC = "/isaac_joint_states"
JOINT_STATES_TOPIC = "/joint_states"
CLOCK_TOPIC = "/clock"
ODOM_TOPIC = "/odom"
TF_TOPIC = "/tf"
TF_STATIC_TOPIC = "/tf_static"
BASE_WHEEL_RADIUS = 0.10
BASE_WHEEL_SEPARATION = 0.55
BASE_LEFT_SIGN = -1.0
BASE_RIGHT_SIGN = -1.0

JOINT_ALIASES = {
    "wheel_L": ("wheel_L_joint",),
    "wheel_R": ("wheel_R_joint",),
    "caster_LF": ("caster_LF_joint",),
    "caster_RF": ("caster_RF_joint",),
    "caster_LB": ("caster_LB_joint",),
    "caster_RB": ("caster_RB_joint",),
}


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


def timestamp_name() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def sec_from_msg_time(msg_time) -> float:
    return float(msg_time.sec) + float(msg_time.nanosec) * 1e-9


def yaw_deg_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def classify_cmd(vx: float, wz: float) -> str:
    eps = 1.0e-12
    if abs(vx) <= eps and abs(wz) <= eps:
        return "stop"
    if vx > eps and abs(wz) <= eps:
        return "forward"
    if vx < -eps and abs(wz) <= eps:
        return "backward"
    if abs(vx) <= eps and wz > eps:
        return "left_arc_90"
    if abs(vx) <= eps and wz < -eps:
        return "right_arc_90"
    return "manual"


def event_from_mode(mode: str) -> str:
    return {
        "forward": "cmd_forward",
        "backward": "cmd_backward",
        "left_arc_90": "cmd_left_arc_90",
        "right_arc_90": "cmd_right_arc_90",
        "stop": "cmd_stop",
        "manual": "cmd_manual",
    }.get(mode, "")


def estimated_wheel_cmds(vx: float, wz: float, mode: str) -> tuple[str, str]:
    if mode in ("left_arc_90", "right_arc_90"):
        return "", ""
    left = BASE_LEFT_SIGN * (vx - wz * BASE_WHEEL_SEPARATION * 0.5) / BASE_WHEEL_RADIUS
    right = BASE_RIGHT_SIGN * (vx + wz * BASE_WHEEL_SEPARATION * 0.5) / BASE_WHEEL_RADIUS
    if abs(left) < 1.0e-12:
        left = 0.0
    if abs(right) < 1.0e-12:
        right = 0.0
    return f"{left:.9g}", f"{right:.9g}"


def csv_float(value) -> str:
    if value is None:
        return ""
    try:
        return f"{float(value):.9g}"
    except (TypeError, ValueError):
        return ""


def csv_bool(value) -> str:
    if value is None:
        return ""
    return "1" if bool(value) else "0"


class BaseDriveRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("base_drive_recorder")
        self.args = args
        self.latest_cmd = Twist()
        self.latest_isaac_joint_state: JointState | None = None
        self.latest_joint_state: JointState | None = None
        self.latest_clock: Clock | None = None
        self.latest_debug: dict | None = None
        self.base_pose: tuple[float, float, float, float] | None = None
        self.latest_mode = "stop"
        self.latest_event = "cmd_stop"
        self.latest_symlink_ok = False

        args.out_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = args.out_dir / f"base_drive_trace_{timestamp_name()}.csv"
        self.latest_path = args.out_dir / "base_drive_trace_latest.csv"
        self.csv_file = self.csv_path.open("w", newline="")
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.header())
        self.writer.writeheader()
        self.create_latest_link()

        self.create_subscription(Twist, args.base_cmd_topic, self.on_cmd, 10)
        self.create_subscription(String, args.base_debug_topic, self.on_debug, 10)
        self.create_subscription(JointState, args.isaac_joint_states_topic, self.on_isaac_joint_state, 50)
        self.create_subscription(JointState, args.joint_states_topic, self.on_joint_state, 50)
        self.create_subscription(Clock, args.clock_topic, self.on_clock, 10)
        if Odometry is not None:
            self.create_subscription(Odometry, args.odom_topic, self.on_odom, 10)
        if TFMessage is not None and args.record_tf:
            self.create_subscription(TFMessage, args.tf_topic, self.on_tf, 50)
            self.create_subscription(TFMessage, args.tf_static_topic, self.on_tf, 10)
        self.timer = self.create_timer(1.0 / args.rate, self.write_row)

        print(f"base drive trace: {self.csv_path.resolve()}")
        print(f"latest: {self.latest_path.resolve()}")

    @staticmethod
    def header() -> list[str]:
        return [
            "wall_time",
            "ros_time_sec",
            "sim_time_sec",
            "base_cmd_vx",
            "base_cmd_wz",
            "base_cmd_mode",
            "base_motion_event",
            "base_left_wheel_cmd",
            "base_right_wheel_cmd",
            "base_debug_mode",
            "base_arc_active",
            "base_arc_armed",
            "base_arc_dir",
            "base_arc_target_yaw_deg",
            "base_arc_current_yaw_deg",
            "base_arc_error_deg",
            "base_script_cmd_vx",
            "base_script_cmd_wz",
            "base_raw_vx",
            "base_raw_wz",
            "base_arc_radius",
            "base_arc_linear_speed",
            "base_debug_frame",
            "base_debug_stamp_wall_time",
            "base_yaw_ok",
            "wheel_L_position",
            "wheel_R_position",
            "wheel_L_velocity",
            "wheel_R_velocity",
            "wheel_L_effort",
            "wheel_R_effort",
            "caster_LF_position",
            "caster_RF_position",
            "caster_LB_position",
            "caster_RB_position",
            "caster_LF_velocity",
            "caster_RF_velocity",
            "caster_LB_velocity",
            "caster_RB_velocity",
            "base_x",
            "base_y",
            "base_z",
            "base_yaw_deg",
            "base_wheel_slip_hint",
        ]

    def create_latest_link(self) -> None:
        try:
            if self.latest_path.exists() or self.latest_path.is_symlink():
                self.latest_path.unlink()
            os.symlink(self.csv_path.name, self.latest_path)
            self.latest_symlink_ok = True
        except OSError:
            self.latest_symlink_ok = False

    def close(self) -> None:
        self.csv_file.flush()
        self.csv_file.close()
        if not self.latest_symlink_ok:
            shutil.copy2(self.csv_path, self.latest_path)

    def on_cmd(self, msg: Twist) -> None:
        self.latest_cmd = msg
        self.latest_mode = classify_cmd(float(msg.linear.x), float(msg.angular.z))
        self.latest_event = event_from_mode(self.latest_mode)

    def on_isaac_joint_state(self, msg: JointState) -> None:
        self.latest_isaac_joint_state = msg

    def on_joint_state(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def on_clock(self, msg: Clock) -> None:
        self.latest_clock = msg

    def on_debug(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.latest_debug = payload

    def on_odom(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.base_pose = (float(p.x), float(p.y), float(p.z), yaw_deg_from_quat(q.x, q.y, q.z, q.w))

    def on_tf(self, msg) -> None:
        for transform in msg.transforms:
            if transform.child_frame_id != self.args.base_frame:
                continue
            p = transform.transform.translation
            q = transform.transform.rotation
            self.base_pose = (float(p.x), float(p.y), float(p.z), yaw_deg_from_quat(q.x, q.y, q.z, q.w))

    def joint_value(self, logical_name: str, field: str) -> str:
        for state in (self.latest_isaac_joint_state, self.latest_joint_state):
            value = self._joint_value_from_state(state, logical_name, field)
            if value != "":
                return value
        return ""

    @staticmethod
    def _joint_value_from_state(state: JointState | None, logical_name: str, field: str) -> str:
        if state is None:
            return ""
        names = list(state.name)
        values = getattr(state, field)
        if not values:
            return ""
        for alias in JOINT_ALIASES[logical_name]:
            try:
                index = names.index(alias)
            except ValueError:
                continue
            if index >= len(values):
                return ""
            return f"{float(values[index]):.9g}"
        return ""

    def write_row(self) -> None:
        now_msg = self.get_clock().now()
        sim_time = "" if self.latest_clock is None else f"{sec_from_msg_time(self.latest_clock.clock):.9f}"
        vx = float(self.latest_cmd.linear.x)
        wz = float(self.latest_cmd.angular.z)
        base_left_wheel_cmd, base_right_wheel_cmd = estimated_wheel_cmds(vx, wz, self.latest_mode)
        debug = self.latest_debug or {}
        if "left_wheel_cmd" in debug or "right_wheel_cmd" in debug:
            base_left_wheel_cmd = csv_float(debug.get("left_wheel_cmd"))
            base_right_wheel_cmd = csv_float(debug.get("right_wheel_cmd"))
        row = {
            "wall_time": f"{time.time():.9f}",
            "ros_time_sec": f"{now_msg.nanoseconds / 1e9:.9f}",
            "sim_time_sec": sim_time,
            "base_cmd_vx": f"{vx:.9g}",
            "base_cmd_wz": f"{wz:.9g}",
            "base_cmd_mode": self.latest_mode,
            "base_motion_event": self.latest_event,
            "base_left_wheel_cmd": base_left_wheel_cmd,
            "base_right_wheel_cmd": base_right_wheel_cmd,
            "base_debug_mode": str(debug.get("mode", "")),
            "base_arc_active": csv_bool(debug.get("arc_active")),
            "base_arc_armed": csv_bool(debug.get("arc_armed")),
            "base_arc_dir": csv_float(debug.get("arc_dir")),
            "base_arc_target_yaw_deg": csv_float(debug.get("target_yaw_deg")),
            "base_arc_current_yaw_deg": csv_float(debug.get("current_yaw_deg")),
            "base_arc_error_deg": csv_float(debug.get("error_yaw_deg")),
            "base_script_cmd_vx": csv_float(debug.get("cmd_vx")),
            "base_script_cmd_wz": csv_float(debug.get("cmd_wz")),
            "base_raw_vx": csv_float(debug.get("raw_vx")),
            "base_raw_wz": csv_float(debug.get("raw_wz")),
            "base_arc_radius": csv_float(debug.get("arc_radius")),
            "base_arc_linear_speed": csv_float(debug.get("arc_linear_speed")),
            "base_debug_frame": "" if debug.get("frame") is None else str(debug.get("frame")),
            "base_debug_stamp_wall_time": csv_float(debug.get("stamp_wall_time")),
            "base_yaw_ok": csv_bool(debug.get("yaw_ok")),
            "wheel_L_position": self.joint_value("wheel_L", "position"),
            "wheel_R_position": self.joint_value("wheel_R", "position"),
            "wheel_L_velocity": self.joint_value("wheel_L", "velocity"),
            "wheel_R_velocity": self.joint_value("wheel_R", "velocity"),
            "wheel_L_effort": self.joint_value("wheel_L", "effort"),
            "wheel_R_effort": self.joint_value("wheel_R", "effort"),
            "caster_LF_position": self.joint_value("caster_LF", "position"),
            "caster_RF_position": self.joint_value("caster_RF", "position"),
            "caster_LB_position": self.joint_value("caster_LB", "position"),
            "caster_RB_position": self.joint_value("caster_RB", "position"),
            "caster_LF_velocity": self.joint_value("caster_LF", "velocity"),
            "caster_RF_velocity": self.joint_value("caster_RF", "velocity"),
            "caster_LB_velocity": self.joint_value("caster_LB", "velocity"),
            "caster_RB_velocity": self.joint_value("caster_RB", "velocity"),
            "base_x": "",
            "base_y": "",
            "base_z": "",
            "base_yaw_deg": "",
            "base_wheel_slip_hint": "",
        }
        if self.base_pose is not None:
            row["base_x"] = f"{self.base_pose[0]:.9g}"
            row["base_y"] = f"{self.base_pose[1]:.9g}"
            row["base_z"] = f"{self.base_pose[2]:.9g}"
            row["base_yaw_deg"] = f"{self.base_pose[3]:.9g}"
        self.writer.writerow(row)
        self.csv_file.flush()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Record /sim1/base_diff_cmd and base wheel/caster state to CSV.")
    parser.add_argument("--rate", type=float, default=50.0)
    parser.add_argument("--out-dir", type=Path, default=Path("trace_data/base_raw"))
    parser.add_argument("--base-cmd-topic", default=BASE_CMD_TOPIC)
    parser.add_argument("--base-debug-topic", default=BASE_DEBUG_TOPIC)
    parser.add_argument("--isaac-joint-states-topic", default=ISAAC_JOINT_STATES_TOPIC)
    parser.add_argument("--joint-states-topic", default=JOINT_STATES_TOPIC)
    parser.add_argument("--clock-topic", default=CLOCK_TOPIC)
    parser.add_argument("--odom-topic", default=ODOM_TOPIC)
    parser.add_argument("--record-tf", action="store_true")
    parser.add_argument("--tf-topic", default=TF_TOPIC)
    parser.add_argument("--tf-static-topic", default=TF_STATIC_TOPIC)
    parser.add_argument("--base-frame", default="base_link")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.rate <= 0.0:
        print("--rate must be > 0", file=sys.stderr)
        return 2

    ensure_ros_log_dir()
    rclpy.init(args=None)
    node = BaseDriveRecorder(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
