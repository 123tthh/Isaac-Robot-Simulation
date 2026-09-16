#!/usr/bin/env python3
"""
Keyboard 3D end-effector teleoperation for OCS2 + RViz2 + Isaac Sim, with logging.

Local documentation referenced:
  - https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
  - https://docs.ros.org/en/rolling/p/rclpy/rclpy.node.md
  - https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Service-And-Client.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_generic_publisher_subscriber.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_tf.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_simulation_control.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/sensors/isaacsim_sensors_physics_effort.md

Scene/control references in this workspace:
  - projects/teleoperation/sim1/base_control/script_node.py
  - projects/teleoperation/sim1/graph/ALL_Graph.md
  - projects/teleoperation/sim1/graph/Gripper_Control_Graph.md
  - data/ros2_log/scripts/ocs2_start.md
  - data/ros2_log/scripts/示教/robot_teach.py
  - data/ros2_log/scripts/夹爪调试/00整理版/POS控制/SCRIPTS NODE/script_gripper_v4.5.py
  - data/ros2_log/scripts/base_control/script_node_base_arc_debug_v2.py
  - projects/ros2_ws/src/arms_ros2_control/libraries/arms_controller_common/src/FSM/StateMoveJ.cpp
  - projects/ros2_ws/src/arms_ros2_control/libraries/arms_controller_common/include/arms_controller_common/FSM/StateMoveJ.h
  - projects/ros2_ws/src/arms_ros2_control/controller/ocs2_arm_controller/src/FSM/StateOCS2.cpp
  - projects/ros2_ws/src/arms_ros2_control/controller/ocs2_arm_controller/src/FSM/StateHold.cpp
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
import shutil
import signal
import select
import subprocess
import sys
import termios
import time
import tty
from copy import deepcopy
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, Int32, String

try:
    from nav_msgs.msg import Odometry
except ImportError:  # pragma: no cover - optional ROS interface
    Odometry = None


LEFT_CURRENT_TOPIC = "/left_current_pose"
RIGHT_CURRENT_TOPIC = "/right_current_pose"
LEFT_TARGET_TOPIC = "/left_target/stamped"
RIGHT_TARGET_TOPIC = "/right_target/stamped"
LEFT_GRIPPER_TOPIC = "/left_gripper_controller/commands"
RIGHT_GRIPPER_TOPIC = "/right_gripper_controller/commands"
JOINT_STATES_TOPIC = "/joint_states"
EFFORT_JOINT_STATES_TOPIC = "/isaac_joint_states"
FSM_TOPIC = "/fsm_command"
BASE_CMD_TOPIC = "/sim1/base_diff_cmd"
BASE_DEBUG_TOPIC = "/sim1/base_drive_debug"
LEFT_RECOVER_JOINT_TOPIC = "/ocs2_arm_controller/target_joint_position/left"
RIGHT_RECOVER_JOINT_TOPIC = "/ocs2_arm_controller/target_joint_position/right"
BASE_WHEEL_RADIUS = 0.10
BASE_WHEEL_SEPARATION = 0.55
BASE_LEFT_SIGN = -1.0
BASE_RIGHT_SIGN = -1.0
CAMERA_TOPICS = {
    "head_rgb": "/head_cam/color/image_raw",
    "head_depth": "/head_cam/depth/image_rect_raw",
    "left_rgb": "/left_cam/color/image_raw",
    "left_depth": "/left_cam/depth/image_rect_raw",
    "right_rgb": "/right_cam/color/image_raw",
    "right_depth": "/right_cam/depth/image_rect_raw",
}
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
LEFT_INITIAL_JOINTS = [
    -1.57086253166,
    1.57073485851,
    -1.57081544399,
    1.5704407692,
    4.17368974013e-05,
    1.56915736198,
    -4.12942608818e-05,
]
RIGHT_INITIAL_JOINTS = [1.5710, 1.5710, -1.5708, -1.5702, 0.0, -1.5677, 3.1416]


HELP = """
Game-style OCS2 teleop

Targets:
  1 left arm     2 right arm     3 base

Position, selected joint7/EE target in base frame:
  w/s  +X/-X     a/d  +Y/-Y      r/f  +Z/-Z

Base, when selected with 3 and --enable-base-motion is active:
  Arrow Up/Down forward/backward, Arrow Left/Right trigger left/right arc 90.

Rotation, selected joint7/EE target:
  i/k  pitch +/-  j/l  yaw +/-   u/o  roll +/-

Gripper:
  Space toggles selected arm gripper
  v/b left close/open    n/m right close/open
  Commands match script_gripper_v4.5.py: >0 opens, <=0 closes/holds

Runtime:
  z recover left arm to initial pose slowly
  x recover right arm to initial pose slowly
  q/Esc finish recording and reset simulator/HOME/targets
  + larger step  - smaller step  p publish current targets  h help

Input modes:
  terminal reads one key event at a time.
  pygame reads held key state and supports W+A/R multi-key velocity control.

Safety gate:
  New target increments are rejected when target-current lag is too large.
  Rotation is also held when orientation error is too large; press z/x to recover the corresponding arm.
  The watchdog only warns and records diagnostics; it does not freeze either arm target.
  安全门限：target-current 误差过大时会拒绝继续远离 current 的移动。
  建议先暂停等待；如需手动修正，按提示沿反方向移动，或按 z/x 慢速恢复对应手臂。
"""


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


def default_ocs2_log_dir() -> Path:
    ros_log_dir = Path(os.environ.get("ROS_LOG_DIR", str(Path.home() / ".ros/log")))
    latest = ros_log_dir / "latest"
    return latest if latest.exists() else ros_log_dir


class Ocs2LogScanner:
    SQP_PATTERN = re.compile(r"SQP did not converge", re.IGNORECASE)
    COST_PATTERN = re.compile(r"\bcost\b[^-+0-9]*(nan|inf|-?\d+(?:\.\d+)?(?:[eE][-+]?\d+)?)", re.IGNORECASE)

    def __init__(self, log_dir: Path, start_time: float, cost_threshold: float) -> None:
        self.log_dir = log_dir
        self.start_time = start_time
        self.cost_threshold = cost_threshold
        self.offsets: dict[Path, int] = {}
        self.cached_files: list[Path] = []
        self.next_file_refresh = 0.0
        self.sqp_not_converged_count = 0
        self.cost_explosion_count = 0
        self.last_cost = math.nan
        self.last_match = ""

    def scan(self) -> None:
        if not self.log_dir.exists():
            return
        for path in self._candidate_files():
            self._scan_file(path)

    def _candidate_files(self) -> list[Path]:
        now = time.monotonic()
        if now < self.next_file_refresh:
            return self.cached_files
        paths = []
        for pattern in ("*.log", "*.txt", "*.stdout", "*.stderr"):
            paths.extend(self.log_dir.rglob(pattern))
        self.cached_files = [path for path in paths if path.is_file()]
        self.next_file_refresh = now + 1.0
        return self.cached_files

    def _scan_file(self, path: Path) -> None:
        try:
            stat = path.stat()
        except OSError:
            return
        if stat.st_mtime + 1.0 < self.start_time:
            return
        offset = self.offsets.get(path, 0)
        if stat.st_size < offset:
            offset = 0
        try:
            with path.open("r", encoding="utf-8", errors="replace") as f:
                f.seek(offset)
                text = f.read()
                self.offsets[path] = f.tell()
        except OSError:
            return
        if not text:
            return

        for line in text.splitlines():
            if self.SQP_PATTERN.search(line):
                self.sqp_not_converged_count += 1
                self.last_match = self._compact(line)
            for match in self.COST_PATTERN.finditer(line):
                raw = match.group(1).lower()
                try:
                    value = float(raw)
                except ValueError:
                    value = math.inf
                self.last_cost = value
                if not math.isfinite(value) or abs(value) >= self.cost_threshold:
                    self.cost_explosion_count += 1
                    self.last_match = self._compact(line)

    @staticmethod
    def _compact(text: str) -> str:
        return " ".join(text.strip().split())[:240]


class CameraRecorderProcess:
    def __init__(self, args: argparse.Namespace, node: Node) -> None:
        self.args = args
        self.node = node
        self.output_dir = args.camera_output_dir
        self.log_path = self.output_dir / "camera_recorder.log"
        self.metadata_path = args.log_csv.with_suffix(".metadata.json")
        self.process: subprocess.Popen | None = None
        self.log_file = None
        self.status = "disabled"
        self.warning = ""
        self.publisher_counts: dict[str, int] = {}

    def start(self) -> None:
        if not self.args.record_cameras:
            self.write_metadata()
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.publisher_counts = self.wait_for_publishers(self.args.camera_startup_grace)
        missing = [name for name, count in self.publisher_counts.items() if count <= 0]
        if missing:
            self.warning = "camera publishers missing: " + ", ".join(missing)
            print(f"[camera] WARNING: {self.warning}", file=sys.stderr)

        cmd = [
            sys.executable,
            str(self.args.camera_recorder_script),
            "--output-root",
            str(self.output_dir.parent),
            "--name",
            self.output_dir.name,
            "--duration",
            "0",
            "--depth-save-every",
            str(self.args.camera_depth_save_every),
        ]
        if self.args.camera_rgb_only:
            cmd.append("--rgb-only")
        if self.args.camera_reliability:
            cmd.extend(["--reliability", self.args.camera_reliability])

        try:
            self.log_file = self.log_path.open("w", encoding="utf-8")
            self.process = subprocess.Popen(
                cmd,
                stdout=self.log_file,
                stderr=subprocess.STDOUT,
                cwd=str(Path(__file__).resolve().parent),
                start_new_session=True,
            )
        except OSError as exc:
            self.status = "failed_to_start"
            self.warning = f"{self.warning}; failed to start camera recorder: {exc}".strip("; ")
            print(f"[camera] WARNING: {self.warning}", file=sys.stderr)
            self._close_log_file()
            self.write_metadata()
            return

        time.sleep(self.args.camera_startup_grace)
        if self.process.poll() is not None:
            self.status = f"exited_early_{self.process.returncode}"
            self.warning = f"{self.warning}; camera recorder exited early".strip("; ")
            print(f"[camera] WARNING: {self.warning}; see {self.log_path}", file=sys.stderr)
        else:
            self.status = "running"
            print(f"[camera] recording -> {self.output_dir}")
            print(f"[camera] log -> {self.log_path}")
        self.write_metadata()

    def check_publishers(self) -> dict[str, int]:
        counts: dict[str, int] = {}
        wanted = CAMERA_TOPICS
        if self.args.camera_rgb_only:
            wanted = {name: topic for name, topic in CAMERA_TOPICS.items() if name.endswith("_rgb")}
        for name, topic in wanted.items():
            try:
                counts[name] = int(self.node.count_publishers(topic))
            except Exception:
                counts[name] = -1
        return counts

    def wait_for_publishers(self, timeout_sec: float) -> dict[str, int]:
        """Allow Isaac Sim discovery/playback to expose all camera publishers."""
        deadline = time.monotonic() + max(0.0, timeout_sec)
        counts = self.check_publishers()
        while time.monotonic() < deadline:
            if counts and all(count > 0 for count in counts.values()):
                return counts
            try:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            except Exception:
                pass
            counts = self.check_publishers()
        return counts

    def stop(self) -> None:
        if self.process is None:
            self.write_metadata()
            return

        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGINT)
            except OSError:
                pass
            try:
                self.process.wait(timeout=self.args.camera_stop_timeout)
            except subprocess.TimeoutExpired:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2.0)

        returncode = self.process.returncode
        self.status = "stopped" if returncode == 0 else f"stopped_returncode_{returncode}"
        if returncode != 0:
            self.warning = f"{self.warning}; camera recorder returncode={returncode}".strip("; ")
        self._close_log_file()

        if self.args.camera_delete_empty_run and not self.has_recorded_frames():
            try:
                shutil.rmtree(self.output_dir)
                self.status = "deleted_empty_run"
            except OSError as exc:
                self.warning = f"{self.warning}; failed to delete empty camera run: {exc}".strip("; ")

        self.write_metadata()

    def has_recorded_frames(self) -> bool:
        summary_path = self.output_dir / "summary.json"
        if not summary_path.exists():
            return False
        try:
            with summary_path.open("r", encoding="utf-8") as f:
                summary = json.load(f)
        except (OSError, json.JSONDecodeError):
            return False
        streams = summary.get("streams", {})
        return any(int(stream.get("frame_count") or 0) > 0 for stream in streams.values())

    def write_metadata(self) -> None:
        data = {
            "trace_csv": str(self.args.log_csv),
            "diagnostics_csv": str(self.args.diagnostics_csv),
            "camera_recording_requested": bool(self.args.record_cameras),
            "camera_output_dir": str(self.output_dir) if self.args.record_cameras else "",
            "camera_timestamps_csv": str(self.output_dir / "camera_timestamps.csv")
            if self.args.record_cameras
            else "",
            "camera_summary_json": str(self.output_dir / "summary.json") if self.args.record_cameras else "",
            "camera_status": self.status,
            "camera_warning": self.warning,
            "camera_publisher_counts": self.publisher_counts,
            "sync_note": "Align trace and camera data by ROS timestamp and wall timestamp; do not align by frame index.",
        }
        try:
            self.metadata_path.parent.mkdir(parents=True, exist_ok=True)
            with self.metadata_path.open("w", encoding="utf-8") as f:
                json.dump(data, f, indent=2)
        except OSError as exc:
            print(f"[camera] WARNING: failed to write metadata {self.metadata_path}: {exc}", file=sys.stderr)

    def _close_log_file(self) -> None:
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None


class RawKeyboard:
    def __enter__(self) -> "RawKeyboard":
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def read_event(self, timeout: float) -> str | None:
        readable, _, _ = select.select([sys.stdin], [], [], timeout)
        if not readable:
            return None
        ch = sys.stdin.read(1)
        if ch != "\x1b":
            return ch

        seq = [ch]
        while True:
            readable, _, _ = select.select([sys.stdin], [], [], 0.001)
            if not readable:
                break
            seq.append(sys.stdin.read(1))
            if seq[-1].isalpha() or seq[-1] in "~":
                break
        text = "".join(seq)
        arrows = {
            "\x1b[A": "up",
            "\x1b[B": "down",
            "\x1b[C": "right",
            "\x1b[D": "left",
        }
        return arrows.get(text, "esc")


def _normalize_quat(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm <= 1e-12:
        return 0.0, 0.0, 0.0, 1.0
    return tuple(v / norm for v in q)


def _quat_multiply(
    a: tuple[float, float, float, float],
    b: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _axis_angle_quat(axis: str, angle: float) -> tuple[float, float, float, float]:
    half = 0.5 * angle
    s = math.sin(half)
    if axis == "x":
        return s, 0.0, 0.0, math.cos(half)
    if axis == "y":
        return 0.0, s, 0.0, math.cos(half)
    if axis == "z":
        return 0.0, 0.0, s, math.cos(half)
    raise ValueError(axis)


def _pose_quat(msg: PoseStamped) -> tuple[float, float, float, float]:
    q = msg.pose.orientation
    return _normalize_quat((q.x, q.y, q.z, q.w))


def _quat_angle(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> float:
    dot = abs(sum(x * y for x, y in zip(_normalize_quat(a), _normalize_quat(b))))
    dot = min(1.0, max(-1.0, dot))
    return 2.0 * math.acos(dot)


def _yaw_deg_from_quat(q: tuple[float, float, float, float]) -> float:
    x, y, z, w = _normalize_quat(q)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.degrees(math.atan2(siny_cosp, cosy_cosp))


def _csv_float(value) -> str:
    if value is None:
        return ""
    try:
        return f"{float(value):.9g}"
    except (TypeError, ValueError):
        return ""


def _csv_bool(value) -> str:
    if value is None:
        return ""
    return "1" if bool(value) else "0"


def _position_error_xyz(target: PoseStamped, current: PoseStamped) -> tuple[float, float, float]:
    dx = target.pose.position.x - current.pose.position.x
    dy = target.pose.position.y - current.pose.position.y
    dz = target.pose.position.z - current.pose.position.z
    return dx, dy, dz


def _position_error_norm(target: PoseStamped, current: PoseStamped) -> float:
    dx, dy, dz = _position_error_xyz(target, current)
    return math.sqrt(dx * dx + dy * dy + dz * dz)


class KeyboardOcs2Teleop(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("keyboard_ocs2_gripper_teleop", parameter_overrides=[
            rclpy.parameter.Parameter("use_sim_time", value=True)
        ])
        self.args = args
        self.selected = "left"
        self.step = args.step
        self.rot_step = math.radians(args.rot_step_deg)

        self.left_current: PoseStamped | None = None
        self.right_current: PoseStamped | None = None
        self.left_target: PoseStamped | None = None
        self.right_target: PoseStamped | None = None
        self.latest_joint_state: JointState | None = None
        self.latest_effort_joint_state: JointState | None = None
        self.latest_fsm_command: int | None = None
        self.fsm_not_ocs2_count = 0
        self.ocs2_log_scanner = Ocs2LogScanner(
            args.ocs2_log_dir,
            start_time=time.time(),
            cost_threshold=args.ocs2_cost_explosion_threshold,
        )
        self.left_gripper_cmd = 0.0
        self.right_gripper_cmd = 0.0
        self.left_gripper_open = False
        self.right_gripper_open = False
        self.base_cmd_vx = 0.0
        self.base_cmd_wz = 0.0
        self.base_cmd_mode = "stop"
        self.base_motion_event = ""
        self.base_blocked_multi_arrow_count = 0
        self.latest_base_debug: dict | None = None
        self.base_pose: tuple[float, float, float] | None = None
        self.blocked_translation_count = 0
        self.blocked_rotation_count = 0
        self.workspace_clamp_count = 0
        self.hard_lag_block_count = 0
        self.watchdog_freeze_count = 0
        self.watchdog_severe_count = 0
        self.watchdog_joint_limit_count = 0
        self.freeze_right_target = False
        self.frozen_right_target: PoseStamped | None = None
        self.last_gate_message_time = 0.0
        self.last_reach_message_time = 0.0
        self.last_watchdog_message_time = 0.0
        self.last_recover_request_wall_time = {"left": 0.0, "right": 0.0}
        self.workspace_center_xyz: dict[str, tuple[float, float, float]] = {}

        self.left_target_pub = self.create_publisher(PoseStamped, args.left_target_topic, 10)
        self.right_target_pub = self.create_publisher(PoseStamped, args.right_target_topic, 10)
        self.left_gripper_pub = self.create_publisher(Float64MultiArray, args.left_gripper_topic, 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, args.right_gripper_topic, 10)
        self.fsm_pub = self.create_publisher(Int32, args.fsm_topic, 10)
        self.base_cmd_pub = self.create_publisher(Twist, args.base_cmd_topic, 10)
        self.left_recover_joint_pub = self.create_publisher(Float64MultiArray, args.left_recover_joint_topic, 10)
        self.right_recover_joint_pub = self.create_publisher(Float64MultiArray, args.right_recover_joint_topic, 10)

        self.create_subscription(PoseStamped, args.left_current_topic, self._left_current_cb, 10)
        self.create_subscription(PoseStamped, args.right_current_topic, self._right_current_cb, 10)
        self.create_subscription(JointState, args.joint_states_topic, self._joint_state_cb, 50)
        self.create_subscription(JointState, args.effort_joint_states_topic, self._effort_joint_state_cb, 50)
        self.create_subscription(Int32, args.fsm_topic, self._fsm_cb, 10)
        self.create_subscription(String, args.base_debug_topic, self._base_debug_cb, 10)
        if Odometry is not None:
            self.create_subscription(Odometry, args.odom_topic, self._odom_cb, 10)

        args.log_csv.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = args.log_csv.open("w", newline="")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(
            [
                "wall_time",
                "ros_time_sec",
                "selected",
                "event",
                "left_target_xyz",
                "right_target_xyz",
                "left_target_xyzw",
                "right_target_xyzw",
                "left_current_xyz",
                "right_current_xyz",
                "left_current_xyzw",
                "right_current_xyzw",
                "left_gripper_cmd",
                "right_gripper_cmd",
                "left_gripper_closed",
                "right_gripper_closed",
                "joint_names",
                "joint_positions",
                "effort_joint_names",
                "joint_efforts",
                "base_cmd_vx",
                "base_cmd_wz",
                "base_cmd_mode",
                "base_motion_event",
                "base_left_wheel_cmd",
                "base_right_wheel_cmd",
                "base_x",
                "base_y",
                "base_yaw_deg",
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
            ]
        )
        args.diagnostics_csv.parent.mkdir(parents=True, exist_ok=True)
        self.diagnostics_file = args.diagnostics_csv.open("w", newline="")
        self.diagnostics_csv = csv.writer(self.diagnostics_file)
        self.diagnostics_csv.writerow(self.diagnostics_header())
        self.camera_recorder = CameraRecorderProcess(args, self)

    def _left_current_cb(self, msg: PoseStamped) -> None:
        self.left_current = msg
        if self.left_target is None:
            self.left_target = deepcopy(msg)

    def _right_current_cb(self, msg: PoseStamped) -> None:
        self.right_current = msg
        if self.right_target is None:
            self.right_target = deepcopy(msg)

    def _joint_state_cb(self, msg: JointState) -> None:
        self.latest_joint_state = msg

    def _effort_joint_state_cb(self, msg: JointState) -> None:
        self.latest_effort_joint_state = msg

    def _fsm_cb(self, msg: Int32) -> None:
        self.latest_fsm_command = int(msg.data)
        if int(msg.data) != self.args.ocs2_fsm_command:
            self.fsm_not_ocs2_count += 1

    def _base_debug_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if isinstance(payload, dict):
            self.latest_base_debug = payload

    def _odom_cb(self, msg) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.base_pose = (
            float(p.x),
            float(p.y),
            _yaw_deg_from_quat((q.x, q.y, q.z, q.w)),
        )

    @staticmethod
    def classify_base_cmd(vx: float, wz: float) -> str:
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

    def publish_base_cmd(self, vx: float, wz: float, event: str = "") -> None:
        if not self.args.enable_base_motion:
            return
        msg = Twist()
        msg.linear.x = float(vx)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(wz)
        self.base_cmd_pub.publish(msg)
        self.base_cmd_vx = float(vx)
        self.base_cmd_wz = float(wz)
        self.base_cmd_mode = self.classify_base_cmd(self.base_cmd_vx, self.base_cmd_wz)
        self.base_motion_event = event

    def publish_base_stop(self, event: str = "base_stop") -> None:
        self.publish_base_cmd(0.0, 0.0, event)

    def handle_base_arrow_key(self, key: str) -> bool:
        if not self.args.enable_base_motion or self.args.base_turn_mode != "arc90":
            return False
        if key == "up":
            self.publish_base_cmd(self.args.base_speed, 0.0, "base_forward")
        elif key == "down":
            self.publish_base_cmd(-self.args.base_speed, 0.0, "base_backward")
        elif key == "left":
            self.publish_base_cmd(0.0, abs(self.args.base_yaw_rate), "base_left_arc_90")
        elif key == "right":
            self.publish_base_cmd(0.0, -abs(self.args.base_yaw_rate), "base_right_arc_90")
        else:
            return False
        self.log_row(self.base_motion_event)
        return True

    def estimated_base_wheel_cmds(self) -> tuple[str, str]:
        if not self.args.enable_base_motion:
            return "", ""
        debug = self.latest_base_debug or {}
        if "left_wheel_cmd" in debug or "right_wheel_cmd" in debug:
            return _csv_float(debug.get("left_wheel_cmd")), _csv_float(debug.get("right_wheel_cmd"))
        if self.base_cmd_mode in ("left_arc_90", "right_arc_90"):
            return "", ""
        left = BASE_LEFT_SIGN * (self.base_cmd_vx - self.base_cmd_wz * BASE_WHEEL_SEPARATION * 0.5) / BASE_WHEEL_RADIUS
        right = BASE_RIGHT_SIGN * (self.base_cmd_vx + self.base_cmd_wz * BASE_WHEEL_SEPARATION * 0.5) / BASE_WHEEL_RADIUS
        if abs(left) < 1.0e-12:
            left = 0.0
        if abs(right) < 1.0e-12:
            right = 0.0
        return f"{left:.9g}", f"{right:.9g}"

    def wait_for_current_poses(self, timeout: float) -> bool:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.left_target is not None and self.right_target is not None:
                return True
        return False

    def close(self) -> None:
        self.csv_file.close()
        self.diagnostics_file.close()

    def stamp_targets(self) -> None:
        now = self.get_clock().now().to_msg()
        if self.left_target:
            self.left_target.header.stamp = now
        if self.right_target:
            self.right_target.header.stamp = now

    def publish_targets(self) -> None:
        self.stamp_targets()
        if self.left_target:
            self.left_target_pub.publish(self.left_target)
        if self.right_target:
            self.right_target_pub.publish(self.right_target)

    def right_joint1_position(self) -> float:
        if self.latest_joint_state is None:
            return math.nan
        try:
            index = list(self.latest_joint_state.name).index("right_joint1")
        except ValueError:
            return math.nan
        if index >= len(self.latest_joint_state.position):
            return math.nan
        return float(self.latest_joint_state.position[index])

    def runtime_watchdog(self) -> None:
        if self.right_target is None or self.right_current is None:
            return

        pos_err = _position_error_norm(self.right_target, self.right_current)
        ori_err = self.orientation_error_deg(self.right_target, self.right_current)
        rj1 = self.right_joint1_position()

        reasons = []
        if (
            (self.args.watchdog_position_error is not None and pos_err > self.args.watchdog_position_error)
            or (
                self.args.watchdog_orientation_error_deg is not None
                and ori_err > self.args.watchdog_orientation_error_deg
            )
        ):
            reasons.append(
                f"right arm unstable: pos_err={pos_err:.3f} m, ori_err={ori_err:.1f} deg"
            )
        if (
            math.isfinite(rj1)
            and self.args.right_joint1_lower_safety is not None
            and self.args.right_joint1_upper_safety is not None
            and (rj1 < self.args.right_joint1_lower_safety or rj1 > self.args.right_joint1_upper_safety)
        ):
            self.watchdog_joint_limit_count += 1
            reasons.append(
                f"right_joint1 safety limit: q={rj1:.3f} rad outside "
                f"[{self.args.right_joint1_lower_safety:.3f}, {self.args.right_joint1_upper_safety:.3f}]"
            )

        if reasons:
            if time.monotonic() - self.last_watchdog_message_time >= self.args.watchdog_print_period:
                self.watchdog_freeze_count += 1
            self._print_watchdog_message(
                "; ".join(reasons)
                + ". Watchdog warning only; right target is not frozen. "
                "右臂 watchdog 仅告警和记录，不冻结 target；需要回初始姿态时按 x。"
            )

        if (
            (
                self.args.watchdog_severe_position_error is not None
                and pos_err > self.args.watchdog_severe_position_error
            )
            or (
                self.args.watchdog_severe_orientation_error_deg is not None
                and ori_err > self.args.watchdog_severe_orientation_error_deg
            )
        ):
            self.watchdog_severe_count += 1
            self._print_watchdog_message(
                f"right arm diverged seriously: pos_err={pos_err:.3f} m, ori_err={ori_err:.1f} deg. "
                "Do not continue the same motion. Press x to recover right arm if needed. "
                "右臂严重发散，请不要继续同方向示教；需要时按 x 恢复右臂。"
            )

    def clear_right_target_freeze(self) -> None:
        self.freeze_right_target = False
        self.frozen_right_target = None

    def publish_fsm_command(self, command: int) -> None:
        msg = Int32()
        msg.data = int(command)
        self.fsm_pub.publish(msg)

    def publish_fsm_ocs2(self) -> None:
        self.publish_fsm_command(self.args.ocs2_fsm_command)

    def publish_fsm_home(self) -> None:
        self.publish_fsm_command(self.args.home_fsm_command)

    def publish_fsm_hold(self) -> None:
        self.publish_fsm_command(self.args.hold_fsm_command)

    def publish_fsm_movej(self) -> None:
        self.publish_fsm_command(self.args.movej_fsm_command)

    def activate_ocs2_mode(self) -> None:
        self.publish_fsm_hold()
        self.spin_for(0.15)
        self.publish_fsm_ocs2()
        self.spin_for(0.15)

    def publish_gripper(self, side: str, command: float) -> None:
        msg = Float64MultiArray()
        msg.data = [float(command)]
        if side == "left":
            self.left_gripper_cmd = float(command)
            self.left_gripper_open = command > 0.0
            self.publish_gripper_repeated(self.left_gripper_pub, msg)
            return
        if side == "right":
            self.right_gripper_cmd = float(command)
            self.right_gripper_open = command > 0.0
            self.publish_gripper_repeated(self.right_gripper_pub, msg)
            return
        raise ValueError(f"unsupported gripper side: {side}")

    def publish_gripper_repeated(self, publisher, msg: Float64MultiArray) -> None:
        for repeat_index in range(self.args.gripper_command_repeats):
            publisher.publish(msg)
            if repeat_index + 1 < self.args.gripper_command_repeats and self.args.gripper_repeat_interval > 0.0:
                self.spin_for(self.args.gripper_repeat_interval)

    def close_all_grippers(self) -> None:
        self.publish_gripper("left", 0.0)
        self.publish_gripper("right", 0.0)

    def toggle_selected_gripper(self) -> bool:
        if self.selected == "left":
            self.publish_gripper("left", 0.0 if self.left_gripper_open else 1.0)
            return True
        if self.selected == "right":
            self.publish_gripper("right", 0.0 if self.right_gripper_open else 1.0)
            return True
        return False

    def select_control_object(self, selected: str) -> None:
        if selected not in ("left", "right", "base"):
            raise ValueError(f"unsupported control object: {selected}")
        self.selected = selected
        if selected != "base":
            self.publish_base_stop("base_stop")

    def selected_target_pairs(self) -> list[tuple[str, PoseStamped, PoseStamped | None]]:
        pairs: list[tuple[str, PoseStamped, PoseStamped | None]] = []
        if self.selected == "left" and self.left_target:
            pairs.append(("left", self.left_target, self.left_current))
        if self.selected == "right" and self.right_target:
            pairs.append(("right", self.right_target, self.right_current))
        return pairs

    def nudge(self, dx: float, dy: float, dz: float) -> bool:
        if self.selected not in ("left", "right"):
            return False
        pairs = self.selected_target_pairs()
        accepted_any = False
        blocked_sides = []
        blocked_reasons = []
        info_reasons = []

        for side, target, current in pairs:
            if current is None:
                continue
            accepted, reason, clamped_xyz = self._apply_translation_candidate(side, target, current, dx, dy, dz)
            if accepted:
                accepted_any = True
                target.pose.position.x, target.pose.position.y, target.pose.position.z = clamped_xyz
                if reason:
                    info_reasons.append(reason)
            else:
                blocked_sides.append(side)
                blocked_reasons.append(reason)

        if blocked_sides:
            self.blocked_translation_count += len(blocked_sides)
            self._print_gate_message(
                "translation",
                blocked_sides,
                self.args.max_position_error or self.args.hard_position_error or 0.0,
                advice="；".join(blocked_reasons) or self._translation_gate_advice(blocked_sides),
            )
        if info_reasons:
            self._print_reach_message("；".join(info_reasons))
        if accepted_any:
            self.publish_fsm_ocs2()
            self.publish_targets()
        return accepted_any

    def rotate(self, axis: str, angle: float) -> bool:
        if self.selected not in ("left", "right"):
            return False
        delta = _axis_angle_quat(axis, angle)
        pairs = self.selected_target_pairs()
        blocked_sides = [
            side
            for side, target, current_pose in pairs
            if current_pose is not None and not self._rotation_allowed(target, current_pose, delta)
        ]
        if blocked_sides:
            self.blocked_rotation_count += len(blocked_sides)
            self._print_gate_message(
                "rotation",
                blocked_sides,
                math.radians(self.args.max_orientation_error_deg),
                advice=self._rotation_gate_advice(axis, angle),
            )
            return False
        for _, target, _ in pairs:
            q = target.pose.orientation
            current = (q.x, q.y, q.z, q.w)
            q.x, q.y, q.z, q.w = _normalize_quat(_quat_multiply(delta, current))
        if pairs:
            self.publish_fsm_ocs2()
            self.publish_targets()
        return bool(pairs)

    @staticmethod
    def _pose_xyz(msg: PoseStamped) -> tuple[float, float, float]:
        p = msg.pose.position
        return float(p.x), float(p.y), float(p.z)

    @staticmethod
    def _vec_sub(a: tuple[float, float, float], b: tuple[float, float, float]) -> tuple[float, float, float]:
        return a[0] - b[0], a[1] - b[1], a[2] - b[2]

    @staticmethod
    def _vec_norm(v: tuple[float, float, float]) -> float:
        return math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])

    @staticmethod
    def _clamp_vec_norm(v: tuple[float, float, float], max_norm: float) -> tuple[float, float, float]:
        norm = KeyboardOcs2Teleop._vec_norm(v)
        if norm <= max_norm or norm <= 1e-12:
            return v
        scale = max_norm / norm
        return v[0] * scale, v[1] * scale, v[2] * scale

    @staticmethod
    def _clamp_vec_axis(
        v: tuple[float, float, float],
        max_abs: tuple[float, float, float],
    ) -> tuple[float, float, float]:
        return (
            max(-max_abs[0], min(max_abs[0], v[0])),
            max(-max_abs[1], min(max_abs[1], v[1])),
            max(-max_abs[2], min(max_abs[2], v[2])),
        )

    @staticmethod
    def _vec_close(a: tuple[float, float, float], b: tuple[float, float, float]) -> bool:
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2])) <= 1e-9

    def _side_workspace_limits(self, side: str) -> tuple[float, tuple[float, float, float]] | None:
        if side == "left":
            if self.args.left_max_target_offset is None or self.args.left_max_axis_offset is None:
                return None
            return self.args.left_max_target_offset, tuple(self.args.left_max_axis_offset)
        if self.args.right_max_target_offset is None or self.args.right_max_axis_offset is None:
            return None
        return self.args.right_max_target_offset, tuple(self.args.right_max_axis_offset)

    def refresh_workspace_center(self) -> None:
        if self.left_target is not None:
            self.workspace_center_xyz["left"] = self._pose_xyz(self.left_target)
        if self.right_target is not None:
            self.workspace_center_xyz["right"] = self._pose_xyz(self.right_target)

    def _workspace_clamp_xyz(
        self,
        side: str,
        candidate: tuple[float, float, float],
    ) -> tuple[tuple[float, float, float], bool, str]:
        center = self.workspace_center_xyz.get(side)
        if center is None:
            return candidate, False, ""
        limits = self._side_workspace_limits(side)
        if limits is None:
            return candidate, False, ""
        max_norm, max_axis = limits
        raw_offset = self._vec_sub(candidate, center)
        sphere_offset = self._clamp_vec_norm(raw_offset, max_norm)
        clamped_offset = self._clamp_vec_axis(sphere_offset, max_axis)
        clamped = (
            center[0] + clamped_offset[0],
            center[1] + clamped_offset[1],
            center[2] + clamped_offset[2],
        )
        workspace_clamped = not self._vec_close(candidate, clamped)
        description = ""
        if workspace_clamped:
            description = self._describe_workspace_clamp(side, clamped_offset, max_axis, max_norm)
        return clamped, workspace_clamped, description

    def _describe_workspace_clamp(
        self,
        side: str,
        offset: tuple[float, float, float],
        axis_limit: tuple[float, float, float],
        sphere_limit: float,
    ) -> str:
        hit = []
        hit_arm = []
        for value, limit, name in zip(offset, axis_limit, ("X", "Y", "Z")):
            if value >= limit - self.args.workspace_hit_epsilon:
                label = f"{name}+"
                hit.append(label)
                hit_arm.append(self._arm_symmetric_axis_label(side, label))
            elif value <= -limit + self.args.workspace_hit_epsilon:
                label = f"{name}-"
                hit.append(label)
                hit_arm.append(self._arm_symmetric_axis_label(side, label))

        norm = self._vec_norm(offset)
        if norm >= sphere_limit - self.args.workspace_hit_epsilon:
            hit.append("SPHERE")
            hit_arm.append("SPHERE")

        return (
            f"{side} workspace clamp: "
            f"offset_from_center=({offset[0]:+.3f}, {offset[1]:+.3f}, {offset[2]:+.3f}) m, "
            f"axis_limit=({axis_limit[0]:.3f}, {axis_limit[1]:.3f}, {axis_limit[2]:.3f}) m, "
            f"sphere_norm={norm:.3f}/{sphere_limit:.3f} m, "
            f"hit_axis={','.join(hit) if hit else 'unknown'}, "
            f"hit_axis_arm={','.join(hit_arm) if hit_arm else 'unknown'}"
        )

    @staticmethod
    def _arm_symmetric_axis_label(side: str, axis_label: str) -> str:
        if axis_label not in ("Y+", "Y-"):
            return axis_label
        if side == "left":
            return "Y_OUT" if axis_label == "Y+" else "Y_IN"
        if side == "right":
            return "Y_OUT" if axis_label == "Y-" else "Y_IN"
        return axis_label

    def _apply_translation_candidate(
        self,
        side: str,
        target: PoseStamped,
        current: PoseStamped,
        dx: float,
        dy: float,
        dz: float,
    ) -> tuple[bool, str, tuple[float, float, float]]:
        previous = self._pose_xyz(target)
        current_xyz = self._pose_xyz(current)
        candidate = previous[0] + dx, previous[1] + dy, previous[2] + dz
        clamped, workspace_clamped, clamp_description = self._workspace_clamp_xyz(side, candidate)

        prev_err = self._vec_norm(self._vec_sub(previous, current_xyz))
        new_err = self._vec_norm(self._vec_sub(clamped, current_xyz))
        if (
            self.args.hard_position_error is not None
            and prev_err > self.args.hard_position_error
            and new_err >= prev_err - self.args.error_gate_epsilon
        ):
            self.hard_lag_block_count += 1
            return (
                False,
                f"{side}: hard lag {prev_err:.3f} m > {self.args.hard_position_error:.3f} m；"
                "目标已经明显不可达，先暂停或按 Z/X 慢速恢复对应手臂",
                previous,
            )

        if self.args.max_position_error is not None and new_err > self.args.max_position_error:
            if new_err < prev_err - self.args.error_gate_epsilon:
                return (
                    True,
                    f"{side}: recovery motion allowed，误差 {prev_err:.3f}->{new_err:.3f} m",
                    clamped,
                )
            return (
                False,
                f"{side}: blocked，candidate 会使 target-current={new_err:.3f} m "
                f"> {self.args.max_position_error:.3f} m；{self._translation_gate_advice([side])}",
                previous,
            )

        if workspace_clamped:
            self.workspace_clamp_count += 1
            return (
                True,
                f"{clamp_description}，target-current={new_err:.3f} m",
                clamped,
            )
        return True, "", clamped

    def _translation_allowed(
        self,
        target: PoseStamped,
        current: PoseStamped,
        dx: float,
        dy: float,
        dz: float,
    ) -> bool:
        old_norm = _position_error_norm(target, current)
        if self.args.max_position_error is None:
            return True
        new_dx = target.pose.position.x + dx - current.pose.position.x
        new_dy = target.pose.position.y + dy - current.pose.position.y
        new_dz = target.pose.position.z + dz - current.pose.position.z
        new_norm = math.sqrt(new_dx * new_dx + new_dy * new_dy + new_dz * new_dz)
        if old_norm <= self.args.max_position_error:
            return new_norm <= self.args.max_position_error or new_norm < old_norm - self.args.error_gate_epsilon
        return new_norm < old_norm - self.args.error_gate_epsilon

    def _rotation_allowed(
        self,
        target: PoseStamped,
        current_pose: PoseStamped,
        delta: tuple[float, float, float, float],
    ) -> bool:
        if self.args.max_orientation_error_deg is None and self.args.max_position_error is None:
            return True
        pos_error = _position_error_norm(target, current_pose)
        target_q = _pose_quat(target)
        current_q = _pose_quat(current_pose)
        old_angle = _quat_angle(target_q, current_q)
        new_angle = _quat_angle(_quat_multiply(delta, target_q), current_q)
        if self.args.max_position_error is not None and pos_error > self.args.max_position_error:
            return False
        if self.args.max_orientation_error_deg is None:
            return True
        max_angle = math.radians(self.args.max_orientation_error_deg)
        if old_angle <= max_angle:
            return new_angle <= max_angle or new_angle < old_angle - self.args.error_gate_epsilon
        return new_angle < old_angle - self.args.error_gate_epsilon

    def _translation_gate_advice(self, blocked_sides: list[str]) -> str:
        lines = []
        for side, target, current in self.selected_target_pairs():
            if side not in blocked_sides or current is None:
                continue
            dx, dy, dz = _position_error_xyz(target, current)
            norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            moves = []
            if abs(dx) >= self.args.translation_advice_axis_threshold:
                moves.append("X- 按 S" if dx > 0.0 else "X+ 按 W")
            if abs(dy) >= self.args.translation_advice_axis_threshold:
                moves.append("Y- 按 D" if dy > 0.0 else "Y+ 按 A")
            if abs(dz) >= self.args.translation_advice_axis_threshold:
                moves.append("Z- 按 F" if dz > 0.0 else "Z+ 按 R")
            move_text = "，".join(moves) if moves else "暂停等待 current 追上"
            lines.append(
                f"{side}: target-current=({dx:+.3f}, {dy:+.3f}, {dz:+.3f}) m, "
                f"|err|={norm:.3f} m；建议 {move_text}"
            )
        return "；".join(lines)

    @staticmethod
    def _rotation_gate_advice(axis: str, angle: float) -> str:
        reverse = {
            ("x", True): "O",
            ("x", False): "U",
            ("y", True): "K",
            ("y", False): "I",
            ("z", True): "L",
            ("z", False): "J",
        }
        key = reverse.get((axis, angle > 0.0), "反向旋转键")
        return f"建议暂停等待姿态误差收敛；若刚才继续旋转导致阻塞，先按 {key} 反向撤回，或按 Z/X 慢速恢复对应手臂"

    def _print_gate_message(
        self,
        kind: str,
        sides: list[str],
        threshold: float,
        advice: str = "",
    ) -> None:
        now = time.monotonic()
        if now - self.last_gate_message_time < self.args.gate_print_period:
            return
        self.last_gate_message_time = now
        sides_text = ", ".join(sides)
        if kind == "translation":
            print(
                f"[gate] blocked {kind} for {sides_text}: target-current lag exceeds "
                f"{threshold:.3f} m. Wait, reverse direction, or press z.\n"
                f"[安全门限] 已阻止{sides_text}平移：target-current 跟踪误差超过 "
                f"{threshold:.3f} m。建议先暂停等待 current 追上；若需要继续操作，"
                f"请沿误差反方向小步移动；仍不恢复时按 Z/X 慢速恢复对应手臂。"
                + (f"\n[建议] {advice}" if advice else "")
            )
        else:
            print(
                f"[gate] blocked {kind} for {sides_text}: pose/orientation error exceeds "
                f"{math.degrees(threshold):.1f} deg. Wait, reverse rotation, or press z.\n"
                f"[安全门限] 已阻止{sides_text}旋转：姿态误差超过 "
                f"{math.degrees(threshold):.1f} deg。建议暂停等待；按反向旋转键回退；"
                f"若误差不收敛，按 Z/X 慢速恢复对应手臂。"
                + (f"\n[建议] {advice}" if advice else "")
            )

    def _print_reach_message(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_reach_message_time < self.args.gate_print_period:
            return
        self.last_reach_message_time = now
        print(f"[reach] {message}")

    def _print_watchdog_message(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_watchdog_message_time < self.args.watchdog_print_period:
            return
        self.last_watchdog_message_time = now
        print(f"[watchdog] {message}")

    def reset_targets_to_current(self) -> None:
        if self.left_current:
            self.left_target = deepcopy(self.left_current)
        if self.right_current:
            self.right_target = deepcopy(self.right_current)
        self.refresh_workspace_center()
        self.publish_targets()

    def reset_after_task(self) -> None:
        print("Resetting grippers, OCS2 targets, and simulator state...")
        self.close_all_grippers()

        if self.args.reset_isaac_sim:
            self.call_isaac_reset_service()

        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < self.args.exit_reset_wait:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.publish_fsm_home()
        for _ in range(3):
            rclpy.spin_once(self, timeout_sec=0.05)
        self.reset_targets_to_current()
        self.log_row("reset_after_task")

    def arm_joint_names_for_side(self, side: str) -> list[str]:
        prefix = "left" if side == "left" else "right"
        return [f"{prefix}_joint{i}" for i in range(1, 8)]

    def initial_joints_for_side(self, side: str) -> list[float]:
        return list(LEFT_INITIAL_JOINTS if side == "left" else RIGHT_INITIAL_JOINTS)

    def current_joints_for_side(self, side: str) -> list[float] | None:
        state = self.latest_joint_state or self.latest_effort_joint_state
        if state is None:
            return None
        by_name = dict(zip(state.name, state.position))
        values: list[float] = []
        for name in self.arm_joint_names_for_side(side):
            if name not in by_name:
                return None
            values.append(float(by_name[name]))
        return values

    def wait_for_side_joint_state(self, side: str, timeout: float) -> list[float] | None:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            values = self.current_joints_for_side(side)
            if values is not None:
                return values
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.current_joints_for_side(side)

    def publish_recover_joint_target(self, side: str, positions: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = [float(v) for v in positions]
        if side == "left":
            self.left_recover_joint_pub.publish(msg)
        else:
            self.right_recover_joint_pub.publish(msg)

    def spin_for(self, duration: float, timeout_step: float = 0.05) -> None:
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=timeout_step)

    def recover_arm_initial_pose(self, side: str) -> bool:
        target = self.initial_joints_for_side(side)
        start = self.wait_for_side_joint_state(side, self.args.recover_state_timeout)
        if start is None:
            print(f"Cannot recover {side} arm: missing joint state for {self.arm_joint_names_for_side(side)}")
            return False

        print(
            f"Recovering {side} arm to initial pose over {self.args.recover_duration:.1f}s "
            "with OCS2 MoveJ..."
        )
        self.publish_base_stop("base_stop")
        self.publish_fsm_hold()
        self.spin_for(self.args.recover_enter_wait)
        self.publish_fsm_movej()
        self.spin_for(self.args.recover_enter_wait)

        period = 1.0 / self.args.recover_rate
        steps = max(1, int(math.ceil(self.args.recover_duration * self.args.recover_rate)))
        for index in range(1, steps + 1):
            if not rclpy.ok():
                return False
            alpha = index / steps
            command = [
                (1.0 - alpha) * start_value + alpha * target_value
                for start_value, target_value in zip(start, target)
            ]
            self.publish_recover_joint_target(side, command)
            rclpy.spin_once(self, timeout_sec=period)

        for _ in range(3):
            self.publish_recover_joint_target(side, target)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.spin_for(self.args.recover_settle_wait)

        self.publish_fsm_hold()
        self.spin_for(self.args.recover_enter_wait)
        self.reset_targets_to_current()
        self.activate_ocs2_mode()
        return True

    def handle_recover_request(self, side: str) -> bool | None:
        now = time.time()
        min_interval = self.args.recover_duration + self.args.recover_settle_wait
        if now - self.last_recover_request_wall_time[side] < min_interval:
            print(f"Ignoring repeated {side} arm recover request; release the key before trying again.")
            return None
        self.last_recover_request_wall_time[side] = now
        return self.recover_arm_initial_pose(side)

    def call_isaac_reset_service(self) -> None:
        try:
            service_type = subprocess.run(
                ["ros2", "service", "type", "/reset_simulation"],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"Isaac reset skipped: cannot query /reset_simulation ({exc})", file=sys.stderr)
            return

        if service_type.returncode != 0 or "simulation_interfaces/srv/ResetSimulation" not in service_type.stdout:
            print(
                "Isaac reset skipped: /reset_simulation is unavailable. "
                "Enable isaacsim.ros2.sim_control and install simulation_interfaces to use it.",
                file=sys.stderr,
            )
            return

        try:
            interface_check = subprocess.run(
                ["ros2", "interface", "show", "simulation_interfaces/srv/ResetSimulation"],
                check=False,
                capture_output=True,
                text=True,
                timeout=2.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            print(f"Isaac reset skipped: cannot inspect ResetSimulation interface ({exc})", file=sys.stderr)
            return
        if interface_check.returncode != 0:
            print(
                "Isaac reset skipped: simulation_interfaces is not installed in this ROS environment.",
                file=sys.stderr,
            )
            return

        try:
            result = subprocess.run(
                [
                    "ros2",
                    "service",
                    "call",
                    "/reset_simulation",
                    "simulation_interfaces/srv/ResetSimulation",
                    "{scope: 255}",
                ],
                check=False,
                capture_output=True,
                text=True,
                timeout=self.args.isaac_reset_timeout,
            )
        except subprocess.TimeoutExpired:
            print("Isaac reset call timed out.", file=sys.stderr)
            return

        if result.returncode != 0:
            detail = (result.stderr or result.stdout).strip()
            print(f"Isaac reset call failed: {detail}", file=sys.stderr)

    @staticmethod
    def xyz(msg: PoseStamped | None) -> str:
        if msg is None:
            return ""
        p = msg.pose.position
        return f"{p.x:.9g} {p.y:.9g} {p.z:.9g}"

    @staticmethod
    def xyzw(msg: PoseStamped | None) -> str:
        if msg is None:
            return ""
        q = msg.pose.orientation
        return f"{q.x:.9g} {q.y:.9g} {q.z:.9g} {q.w:.9g}"

    @staticmethod
    def gripper_closed_text(command: float) -> str:
        if math.isnan(command):
            return ""
        return "1" if command <= 0.0 else "0"

    @staticmethod
    def values_by_name(names: list[str], values: list[float], wanted: list[str]) -> dict[str, float]:
        by_name = dict(zip(names, values))
        return {name: float(by_name[name]) for name in wanted if name in by_name}

    @staticmethod
    def pose_error_xyz(target: PoseStamped | None, current: PoseStamped | None) -> tuple[float, float, float, float]:
        if target is None or current is None:
            return math.nan, math.nan, math.nan, math.nan
        dx = target.pose.position.x - current.pose.position.x
        dy = target.pose.position.y - current.pose.position.y
        dz = target.pose.position.z - current.pose.position.z
        return dx, dy, dz, math.sqrt(dx * dx + dy * dy + dz * dz)

    @staticmethod
    def diagnostics_header() -> list[str]:
        columns = [
            "wall_time",
            "ros_time_sec",
            "event",
            "selected",
            "fsm_command",
            "fsm_is_ocs2",
            "fsm_not_ocs2_count",
            "left_target_current_dx",
            "left_target_current_dy",
            "left_target_current_dz",
            "left_target_current_norm",
            "right_target_current_dx",
            "right_target_current_dy",
            "right_target_current_dz",
            "right_target_current_norm",
            "left_orientation_error_deg",
            "right_orientation_error_deg",
            "blocked_translation_count",
            "blocked_rotation_count",
            "workspace_clamp_count",
            "hard_lag_block_count",
            "watchdog_freeze_right",
            "watchdog_freeze_count",
            "watchdog_severe_count",
            "watchdog_joint_limit_count",
            "left_gripper_closed",
            "right_gripper_closed",
            "ocs2_sqp_not_converged_count",
            "ocs2_cost_explosion_count",
            "ocs2_last_cost",
            "ocs2_last_log_match",
            "camera_recorder_status",
            "camera_output_dir",
            "camera_warning",
            "base_motion_enabled",
            "base_blocked_multi_arrow_count",
            "base_arc_active",
            "base_arc_armed",
            "base_arc_dir",
            "base_arc_target_yaw_deg",
            "base_arc_current_yaw_deg",
            "base_arc_error_deg",
            "base_debug_mode",
            "base_script_cmd_vx",
            "base_script_cmd_wz",
            "base_raw_vx",
            "base_raw_wz",
            "base_debug_frame",
            "base_debug_stamp_wall_time",
            "base_yaw_ok",
            "base_wheel_slip_hint",
        ]
        for prefix in ("left", "right"):
            for i in range(1, 8):
                columns.append(f"{prefix}_joint{i}_position")
        for prefix in ("left", "right"):
            for i in range(1, 8):
                columns.append(f"{prefix}_joint{i}_effort")
        return columns

    def log_row(self, event: str) -> None:
        js_names = ""
        js_pos = ""
        effort_names = ""
        js_effort = ""
        if self.latest_joint_state:
            js_names = " ".join(self.latest_joint_state.name)
            js_pos = " ".join(f"{float(v):.9g}" for v in self.latest_joint_state.position)
        effort_state = self.latest_effort_joint_state or self.latest_joint_state
        if effort_state:
            effort_names = " ".join(effort_state.name)
            js_effort = " ".join(f"{float(v):.9g}" for v in effort_state.effort)
        base_left_wheel_cmd, base_right_wheel_cmd = self.estimated_base_wheel_cmds()
        base_x = base_y = base_yaw_deg = ""
        debug = self.latest_base_debug or {}
        if self.base_pose is not None:
            base_x = f"{self.base_pose[0]:.9g}"
            base_y = f"{self.base_pose[1]:.9g}"
            base_yaw_deg = f"{self.base_pose[2]:.9g}"

        self.csv.writerow(
            [
                f"{time.time():.9f}",
                f"{self.get_clock().now().nanoseconds / 1e9:.9f}",
                self.selected,
                event,
                self.xyz(self.left_target),
                self.xyz(self.right_target),
                self.xyzw(self.left_target),
                self.xyzw(self.right_target),
                self.xyz(self.left_current),
                self.xyz(self.right_current),
                self.xyzw(self.left_current),
                self.xyzw(self.right_current),
                f"{self.left_gripper_cmd:.9g}",
                f"{self.right_gripper_cmd:.9g}",
                self.gripper_closed_text(self.left_gripper_cmd),
                self.gripper_closed_text(self.right_gripper_cmd),
                js_names,
                js_pos,
                effort_names,
                js_effort,
                f"{self.base_cmd_vx:.9g}",
                f"{self.base_cmd_wz:.9g}",
                self.base_cmd_mode,
                self.base_motion_event,
                base_left_wheel_cmd,
                base_right_wheel_cmd,
                base_x,
                base_y,
                base_yaw_deg,
                str(debug.get("mode", "")),
                _csv_bool(debug.get("arc_active")),
                _csv_bool(debug.get("arc_armed")),
                _csv_float(debug.get("arc_dir")),
                _csv_float(debug.get("target_yaw_deg")),
                _csv_float(debug.get("current_yaw_deg")),
                _csv_float(debug.get("error_yaw_deg")),
                _csv_float(debug.get("cmd_vx")),
                _csv_float(debug.get("cmd_wz")),
                _csv_float(debug.get("raw_vx")),
                _csv_float(debug.get("raw_wz")),
                _csv_float(debug.get("arc_radius")),
                _csv_float(debug.get("arc_linear_speed")),
                "" if debug.get("frame") is None else str(debug.get("frame")),
                _csv_float(debug.get("stamp_wall_time")),
                _csv_bool(debug.get("yaw_ok")),
            ]
        )
        self.csv_file.flush()
        self.log_diagnostics_row(event)

    def log_diagnostics_row(self, event: str) -> None:
        self.ocs2_log_scanner.scan()
        now = f"{time.time():.9f}"
        ros_now = f"{self.get_clock().now().nanoseconds / 1e9:.9f}"
        left_err = self.pose_error_xyz(self.left_target, self.left_current)
        right_err = self.pose_error_xyz(self.right_target, self.right_current)
        left_orientation_error = self.orientation_error_deg(self.left_target, self.left_current)
        right_orientation_error = self.orientation_error_deg(self.right_target, self.right_current)

        joint_positions = {name: math.nan for name in ARM_JOINTS}
        if self.latest_joint_state:
            joint_positions.update(
                self.values_by_name(
                    list(self.latest_joint_state.name),
                    [float(v) for v in self.latest_joint_state.position],
                    ARM_JOINTS,
                )
            )

        joint_efforts = {name: math.nan for name in ARM_JOINTS}
        effort_state = self.latest_effort_joint_state or self.latest_joint_state
        if effort_state:
            joint_efforts.update(
                self.values_by_name(
                    list(effort_state.name),
                    [float(v) for v in effort_state.effort],
                    ARM_JOINTS,
                )
            )

        fsm_command = "" if self.latest_fsm_command is None else str(self.latest_fsm_command)
        fsm_is_ocs2 = "" if self.latest_fsm_command is None else (
            "1" if self.latest_fsm_command == self.args.ocs2_fsm_command else "0"
        )
        debug = self.latest_base_debug or {}
        row: list[str] = [
            now,
            ros_now,
            event,
            self.selected,
            fsm_command,
            fsm_is_ocs2,
            str(self.fsm_not_ocs2_count),
            *(f"{v:.9g}" for v in left_err),
            *(f"{v:.9g}" for v in right_err),
            f"{left_orientation_error:.9g}",
            f"{right_orientation_error:.9g}",
            str(self.blocked_translation_count),
            str(self.blocked_rotation_count),
            str(self.workspace_clamp_count),
            str(self.hard_lag_block_count),
            "1" if self.freeze_right_target else "0",
            str(self.watchdog_freeze_count),
            str(self.watchdog_severe_count),
            str(self.watchdog_joint_limit_count),
            self.gripper_closed_text(self.left_gripper_cmd),
            self.gripper_closed_text(self.right_gripper_cmd),
            str(self.ocs2_log_scanner.sqp_not_converged_count),
            str(self.ocs2_log_scanner.cost_explosion_count),
            f"{self.ocs2_log_scanner.last_cost:.9g}",
            self.ocs2_log_scanner.last_match,
            self.camera_recorder.status,
            str(self.camera_recorder.output_dir) if self.args.record_cameras else "",
            self.camera_recorder.warning,
            "1" if self.args.enable_base_motion else "0",
            str(self.base_blocked_multi_arrow_count),
            _csv_bool(debug.get("arc_active")),
            _csv_bool(debug.get("arc_armed")),
            _csv_float(debug.get("arc_dir")),
            _csv_float(debug.get("target_yaw_deg")),
            _csv_float(debug.get("current_yaw_deg")),
            _csv_float(debug.get("error_yaw_deg")),
            str(debug.get("mode", "")),
            _csv_float(debug.get("cmd_vx")),
            _csv_float(debug.get("cmd_wz")),
            _csv_float(debug.get("raw_vx")),
            _csv_float(debug.get("raw_wz")),
            "" if debug.get("frame") is None else str(debug.get("frame")),
            _csv_float(debug.get("stamp_wall_time")),
            _csv_bool(debug.get("yaw_ok")),
            "",
        ]
        row.extend(f"{joint_positions[name]:.9g}" for name in ARM_JOINTS)
        row.extend(f"{joint_efforts[name]:.9g}" for name in ARM_JOINTS)
        self.diagnostics_csv.writerow(row)
        self.diagnostics_file.flush()

    @staticmethod
    def orientation_error_deg(target: PoseStamped | None, current: PoseStamped | None) -> float:
        if target is None or current is None:
            return math.nan
        return math.degrees(_quat_angle(_pose_quat(target), _pose_quat(current)))

    def handle_key(self, key: str) -> bool:
        event = key
        key = key.lower() if len(key) == 1 else key
        if key in ("q", "esc", "\x03"):
            self.log_row("quit")
            return False
        target_motion_keys = {
            "w", "s", "a", "d", "r", "f",
            "u", "o", "i", "k", "j", "l",
        }
        if self.selected == "base" and key in ("up", "down", "left", "right") and self.handle_base_arrow_key(key):
            return True
        if key == "h":
            print(HELP)
        elif key == "1":
            self.select_control_object("left")
        elif key == "2":
            self.select_control_object("right")
        elif key == "3":
            self.select_control_object("base")
        elif key == "+" or key == "=":
            self.step *= 2.0
            print(f"step={self.step:.5f} m")
        elif key == "-" or key == "_":
            self.step = max(self.step * 0.5, 1e-5)
            print(f"step={self.step:.5f} m")
        elif key == "p":
            self.publish_targets()
        elif key == " ":
            event = "toggle_gripper" if self.toggle_selected_gripper() else "toggle_gripper_ignored"
        elif key == "w":
            if not self.nudge(self.step, 0.0, 0.0):
                event = "blocked_translation"
        elif key == "s":
            if not self.nudge(-self.step, 0.0, 0.0):
                event = "blocked_translation"
        elif key == "a":
            if not self.nudge(0.0, self.step, 0.0):
                event = "blocked_translation"
        elif key == "d":
            if not self.nudge(0.0, -self.step, 0.0):
                event = "blocked_translation"
        elif key == "r":
            if not self.nudge(0.0, 0.0, self.step):
                event = "blocked_translation"
        elif key == "f":
            if not self.nudge(0.0, 0.0, -self.step):
                event = "blocked_translation"
        elif key == "u":
            if not self.rotate("x", self.rot_step):
                event = "blocked_rotation"
        elif key == "o":
            if not self.rotate("x", -self.rot_step):
                event = "blocked_rotation"
        elif key == "i":
            if not self.rotate("y", self.rot_step):
                event = "blocked_rotation"
        elif key == "k":
            if not self.rotate("y", -self.rot_step):
                event = "blocked_rotation"
        elif key == "j":
            if not self.rotate("z", self.rot_step):
                event = "blocked_rotation"
        elif key == "l":
            if not self.rotate("z", -self.rot_step):
                event = "blocked_rotation"
        elif key == "z":
            recovered = self.handle_recover_request("left")
            if recovered is None:
                return True
            event = "recover_left_initial" if recovered else "recover_left_failed"
        elif key == "x":
            recovered = self.handle_recover_request("right")
            if recovered is None:
                return True
            event = "recover_right_initial" if recovered else "recover_right_failed"
        elif key == "v":
            self.publish_gripper("left", 0.0)
            event = "left_close"
        elif key == "b":
            self.publish_gripper("left", 1.0)
            event = "left_open"
        elif key == "n":
            self.publish_gripper("right", 0.0)
            event = "right_close"
        elif key == "m":
            self.publish_gripper("right", 1.0)
            event = "right_open"
        else:
            return True

        self.log_row(event)
        return True

    @staticmethod
    def _normalized_motion(v: tuple[float, float, float]) -> tuple[float, float, float]:
        norm = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        if norm <= 1e-12:
            return 0.0, 0.0, 0.0
        return v[0] / norm, v[1] / norm, v[2] / norm

    def _right_motion_frozen(self) -> bool:
        return False

    def _run_terminal_loop(self) -> None:
        next_tick = time.monotonic()
        period = 1.0 / self.args.rate
        with RawKeyboard() as keyboard:
            while rclpy.ok():
                now = time.monotonic()
                timeout = max(0.0, min(period, next_tick - now))
                key = keyboard.read_event(timeout)
                rclpy.spin_once(self, timeout_sec=0.0)
                if key and not self.handle_key(key):
                    break

                if time.monotonic() >= next_tick:
                    self.runtime_watchdog()
                    if self.args.enable_base_motion:
                        self.publish_base_stop("base_stop")
                    self.publish_targets()
                    self.log_row("tick")
                    next_tick += period

    def _pygame_motion_vectors(self, pressed) -> tuple[tuple[float, float, float], tuple[float, float, float]]:
        import pygame

        if self.selected not in ("left", "right"):
            return (0.0, 0.0, 0.0), (0.0, 0.0, 0.0)

        tx = (1.0 if pressed[pygame.K_w] else 0.0) + (-1.0 if pressed[pygame.K_s] else 0.0)
        ty = (1.0 if pressed[pygame.K_a] else 0.0) + (-1.0 if pressed[pygame.K_d] else 0.0)
        tz = (1.0 if pressed[pygame.K_r] else 0.0) + (-1.0 if pressed[pygame.K_f] else 0.0)
        rx = (1.0 if pressed[pygame.K_u] else 0.0) + (-1.0 if pressed[pygame.K_o] else 0.0)
        ry = (1.0 if pressed[pygame.K_i] else 0.0) + (-1.0 if pressed[pygame.K_k] else 0.0)
        rz = (1.0 if pressed[pygame.K_j] else 0.0) + (-1.0 if pressed[pygame.K_l] else 0.0)
        return self._normalized_motion((tx, ty, tz)), self._normalized_motion((rx, ry, rz))

    def _pygame_base_arrow_event(self, pressed) -> str:
        if not self.args.enable_base_motion:
            return ""
        if self.selected != "base":
            self.publish_base_stop("base_stop")
            return ""
        import pygame

        arrows = [
            ("up", bool(pressed[pygame.K_UP])),
            ("down", bool(pressed[pygame.K_DOWN])),
            ("left", bool(pressed[pygame.K_LEFT])),
            ("right", bool(pressed[pygame.K_RIGHT])),
        ]
        active = [name for name, is_pressed in arrows if is_pressed]
        if len(active) > 1:
            self.base_blocked_multi_arrow_count += 1
            self.publish_base_stop("base_blocked_multi_arrow")
            return "base_blocked_multi_arrow"
        if not active:
            self.publish_base_stop("base_stop")
            return ""
        name = active[0]
        if name == "up":
            self.publish_base_cmd(self.args.base_speed, 0.0, "base_forward")
        elif name == "down":
            self.publish_base_cmd(-self.args.base_speed, 0.0, "base_backward")
        elif name == "left":
            self.publish_base_cmd(0.0, abs(self.args.base_yaw_rate), "base_left_arc_90")
        elif name == "right":
            self.publish_base_cmd(0.0, -abs(self.args.base_yaw_rate), "base_right_arc_90")
        return self.base_motion_event

    def _handle_pygame_keydown(self, key: int) -> bool:
        import pygame

        key_map = {
            pygame.K_1: "1",
            pygame.K_2: "2",
            pygame.K_3: "3",
            pygame.K_z: "z",
            pygame.K_p: "p",
            pygame.K_h: "h",
            pygame.K_b: "b",
            pygame.K_x: "x",
            pygame.K_n: "n",
            pygame.K_m: "m",
            pygame.K_v: "v",
            pygame.K_SPACE: " ",
            pygame.K_EQUALS: "+",
            pygame.K_MINUS: "-",
        }
        if hasattr(pygame, "K_PLUS"):
            key_map[pygame.K_PLUS] = "+"
        if hasattr(pygame, "K_UNDERSCORE"):
            key_map[pygame.K_UNDERSCORE] = "-"
        if key in (pygame.K_q, pygame.K_ESCAPE):
            self.log_row("quit")
            return False
        mapped = key_map.get(key)
        if mapped is not None:
            return self.handle_key(mapped)
        return True

    @staticmethod
    def _pygame_font(size: int):
        import pygame

        for name in (
            "Noto Sans CJK SC",
            "Noto Sans CJK JP",
            "WenQuanYi Zen Hei",
            "Microsoft YaHei",
            "SimHei",
            "DejaVu Sans",
        ):
            path = pygame.font.match_font(name)
            if path:
                return pygame.font.Font(path, size)
        return pygame.font.Font(None, size)

    def _run_pygame_loop(self) -> int:
        try:
            import pygame
        except ImportError:
            print("pygame is not installed. Install python3-pygame or use --input-mode terminal.", file=sys.stderr)
            return 2

        pygame.init()
        try:
            pygame.display.set_caption("SIM1 OCS2 双臂夹爪底盘示教")
            screen = pygame.display.set_mode((1180, 330), pygame.RESIZABLE)
        except pygame.error as exc:
            print(
                f"pygame display could not be opened ({exc}). "
                "Use --input-mode terminal or run from a graphical session.",
                file=sys.stderr,
            )
            pygame.quit()
            return 2
        title_font = self._pygame_font(24)
        font = self._pygame_font(21)
        clock = pygame.time.Clock()
        running = True
        started = time.monotonic()

        while running and rclpy.ok():
            elapsed = time.monotonic() - started
            if self.args.duration > 0 and elapsed >= self.args.duration:
                self.log_row("duration_complete")
                break
            dt = clock.tick(self.args.control_rate) / 1000.0
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.log_row("quit")
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if not self._handle_pygame_keydown(event.key):
                        running = False
            if not running:
                break

            rclpy.spin_once(self, timeout_sec=0.0)
            self.runtime_watchdog()
            pressed = pygame.key.get_pressed()
            base_event = None if self.args.validation_motion else self._pygame_base_arrow_event(pressed)
            trans_dir, rot_dir = self._pygame_motion_vectors(pressed)
            event_name = "tick"

            if trans_dir != (0.0, 0.0, 0.0):
                delta = self.args.linear_speed * dt
                if self.nudge(trans_dir[0] * delta, trans_dir[1] * delta, trans_dir[2] * delta):
                    event_name = "pygame_translation"
                else:
                    event_name = "blocked_translation"
            if rot_dir != (0.0, 0.0, 0.0):
                delta = math.radians(self.args.angular_speed_deg) * dt
                moved_rotation = False
                if abs(rot_dir[0]) > 1e-12:
                    moved_rotation = self.rotate("x", rot_dir[0] * delta) or moved_rotation
                if abs(rot_dir[1]) > 1e-12:
                    moved_rotation = self.rotate("y", rot_dir[1] * delta) or moved_rotation
                if abs(rot_dir[2]) > 1e-12:
                    moved_rotation = self.rotate("z", rot_dir[2] * delta) or moved_rotation
                if moved_rotation:
                    event_name = "pygame_rotation" if event_name == "tick" else "pygame_motion"
                elif event_name == "tick":
                    event_name = "blocked_rotation"

            if base_event and event_name == "tick":
                event_name = base_event
            if self.args.validation_motion:
                # Bounded ten-second integration exercise through the same
                # command path as keyboard control; all feedback remains real.
                if 1.0 <= elapsed < 3.0:
                    self.selected = "base"
                    self.publish_base_cmd(0.05, 0.0, "base_forward")
                    event_name = "validation_base_forward"
                else:
                    self.publish_base_stop("base_stop")
                    if 4.0 <= elapsed < 8.0:
                        self.selected = "left" if elapsed < 6.0 else "right"
                        if self.nudge(0.005 * dt, 0.0, 0.0):
                            event_name = "validation_translation"
            self.publish_targets()
            self.log_row(event_name)
            screen.fill((20, 24, 28))
            selected_label = {
                "left": "左臂",
                "right": "右臂",
                "base": "底盘",
            }.get(self.selected, self.selected)
            lines = [
                ("title", "SIM1 OCS2 双臂/夹爪/底盘示教"),
                ("normal", f"夹爪选择/控制对象: 1 左臂    2 右臂    3 底盘    当前: {selected_label}"),
                ("normal", f"手臂运动: W/S X轴    A/D Y轴    R/F Z轴    U/O I/K J/L 旋转"),
                ("normal", "夹爪操控: V 左闭    B 左开    N 右闭    M 右开    空格切换选中臂"),
                (
                    "normal",
                    "底盘运动: 选 3 后  ↑ 前进    ↓ 后退    ← 左转90    → 右转90"
                    f"    vx={self.base_cmd_vx:.2f} wz={self.base_cmd_wz:.2f} {self.base_cmd_mode}",
                ),
                ("normal", "恢复/退出: Z 左臂回初始    X 右臂回初始    Q/Esc 结束记录"),
            ]
            y = 18
            for kind, line in lines:
                active_font = title_font if kind == "title" else font
                color = (255, 210, 120) if kind == "warning" else (235, 235, 235)
                screen.blit(active_font.render(line, True, color), (20, y))
                y += 34 if kind == "title" else 30
            pygame.display.flip()

        pygame.quit()
        return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Keyboard teleop for /left_target/stamped and /right_target/stamped."
    )
    parser.add_argument("--step", type=float, default=0.003, help="Initial xyz step in meters.")
    parser.add_argument("--rot-step-deg", type=float, default=0.5, help="Initial rotation step in degrees.")
    parser.add_argument("--rate", type=float, default=20.0, help="Target republish and log rate.")
    parser.add_argument(
        "--input-mode",
        choices=("terminal", "pygame"),
        default="terminal",
        help="terminal reads one key event; pygame reads held key state for multi-key velocity control.",
    )
    parser.add_argument("--linear-speed", type=float, default=0.025, help="Pygame mode linear speed in m/s.")
    parser.add_argument(
        "--angular-speed-deg",
        type=float,
        default=4.0,
        help="Pygame mode angular speed in deg/s.",
    )
    parser.add_argument("--control-rate", type=float, default=50.0, help="Pygame mode control loop rate in Hz.")
    parser.set_defaults(enable_base_motion=True)
    parser.add_argument("--enable-base-motion", dest="enable_base_motion", action="store_true")
    parser.add_argument("--disable-base-motion", dest="enable_base_motion", action="store_false")
    parser.add_argument("--base-speed", type=float, default=0.25, help="Base forward/backward speed in m/s.")
    parser.add_argument("--base-yaw-rate", type=float, default=1.0, help="Base arc-90 angular.z trigger magnitude.")
    parser.add_argument("--base-turn-mode", choices=("arc90",), default="arc90")
    parser.add_argument("--wait-timeout", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=0.0,
                        help="Stop pygame recording after this many wall seconds; 0 is interactive.")
    parser.add_argument("--validation-motion", action="store_true",
                        help="Run a bounded base/left/right motion sequence; requires --duration 10.")
    parser.add_argument("--no-activate-ocs2", action="store_true")
    parser.add_argument("--reset-targets", action="store_true", help="Reset targets to current poses before teleop.")
    parser.add_argument("--no-reset-on-exit", action="store_true", help="Do not reset targets/grippers/sim when quitting.")
    parser.add_argument("--reset-isaac-sim", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--isaac-reset-timeout", type=float, default=5.0)
    parser.add_argument("--exit-reset-wait", type=float, default=1.0)
    parser.add_argument("--home-fsm-command", type=int, default=1)
    parser.add_argument("--hold-fsm-command", type=int, default=2)
    parser.add_argument("--ocs2-fsm-command", type=int, default=3)
    parser.add_argument("--movej-fsm-command", type=int, default=4)
    parser.add_argument("--recover-duration", type=float, default=6.0)
    parser.add_argument("--recover-rate", type=float, default=10.0)
    parser.add_argument("--recover-enter-wait", type=float, default=0.3)
    parser.add_argument("--recover-settle-wait", type=float, default=1.0)
    parser.add_argument("--recover-state-timeout", type=float, default=2.0)
    parser.add_argument("--log-csv", type=Path, default=Path("manual_ocs2_keyboard_trace.csv"))
    parser.add_argument("--diagnostics-csv", type=Path, default=None)
    parser.add_argument("--ocs2-log-dir", type=Path, default=None)
    parser.add_argument("--ocs2-cost-explosion-threshold", type=float, default=1.0e6)
    parser.add_argument("--max-position-error", type=float, default=None)
    parser.add_argument("--hard-position-error", type=float, default=None)
    parser.add_argument("--left-max-target-offset", type=float, default=None)
    parser.add_argument("--right-max-target-offset", type=float, default=None)
    parser.add_argument("--left-max-axis-offset", nargs=3, type=float, default=None)
    parser.add_argument("--right-max-axis-offset", nargs=3, type=float, default=None)
    parser.add_argument("--max-orientation-error-deg", type=float, default=None)
    parser.add_argument("--error-gate-epsilon", type=float, default=1.0e-6)
    parser.add_argument("--gate-print-period", type=float, default=1.0)
    parser.add_argument("--translation-advice-axis-threshold", type=float, default=0.005)
    parser.add_argument("--workspace-hit-epsilon", type=float, default=1.0e-4)
    parser.add_argument("--watchdog-position-error", type=float, default=None)
    parser.add_argument("--watchdog-orientation-error-deg", type=float, default=None)
    parser.add_argument("--watchdog-severe-position-error", type=float, default=None)
    parser.add_argument("--watchdog-severe-orientation-error-deg", type=float, default=None)
    parser.add_argument("--right-joint1-lower-safety", type=float, default=None)
    parser.add_argument("--right-joint1-upper-safety", type=float, default=None)
    parser.add_argument("--watchdog-print-period", type=float, default=1.0)
    parser.add_argument("--record-cameras", action="store_true", help="Start tools/camera_recorder.py with this trace.")
    parser.add_argument("--camera-rgb-only", action="store_true", help="Record only RGB camera streams.")
    parser.add_argument(
        "--camera-depth-save-every",
        type=int,
        default=1,
        help="Save every Nth depth frame when camera recording is enabled.",
    )
    parser.add_argument(
        "--camera-delete-empty-run",
        action="store_true",
        help="Delete the camera output directory if no stream records any frames.",
    )
    parser.add_argument("--camera-stop-timeout", type=float, default=5.0)
    parser.add_argument(
        "--camera-startup-grace",
        type=float,
        default=5.0,
        help="Seconds to wait for Isaac Sim camera publishers before recording starts.",
    )
    parser.add_argument(
        "--camera-reliability",
        choices=("reliable", "best_effort"),
        default="reliable",
        help="QoS reliability passed to tools/camera_recorder.py (reliable for this project's CameraHelpers).",
    )
    parser.add_argument("--left-current-topic", default=LEFT_CURRENT_TOPIC)
    parser.add_argument("--right-current-topic", default=RIGHT_CURRENT_TOPIC)
    parser.add_argument("--left-target-topic", default=LEFT_TARGET_TOPIC)
    parser.add_argument("--right-target-topic", default=RIGHT_TARGET_TOPIC)
    parser.add_argument("--left-gripper-topic", default=LEFT_GRIPPER_TOPIC)
    parser.add_argument("--right-gripper-topic", default=RIGHT_GRIPPER_TOPIC)
    parser.add_argument(
        "--gripper-command-repeats",
        type=int,
        default=3,
        help="Publish each gripper command this many times to avoid missed Isaac graph frames.",
    )
    parser.add_argument(
        "--gripper-repeat-interval",
        type=float,
        default=0.03,
        help="Seconds between repeated gripper command publishes.",
    )
    parser.add_argument("--joint-states-topic", default=JOINT_STATES_TOPIC)
    parser.add_argument("--effort-joint-states-topic", default=EFFORT_JOINT_STATES_TOPIC)
    parser.add_argument("--fsm-topic", default=FSM_TOPIC)
    parser.add_argument("--base-cmd-topic", default=BASE_CMD_TOPIC)
    parser.add_argument("--base-debug-topic", default=BASE_DEBUG_TOPIC)
    parser.add_argument("--left-recover-joint-topic", default=LEFT_RECOVER_JOINT_TOPIC)
    parser.add_argument("--right-recover-joint-topic", default=RIGHT_RECOVER_JOINT_TOPIC)
    parser.add_argument("--odom-topic", default="/odom")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.validation_motion and (args.input_mode != "pygame" or args.duration != 10):
        raise SystemExit("--validation-motion requires --input-mode pygame --duration 10")
    if args.step <= 0.0:
        print("--step must be > 0", file=sys.stderr)
        return 2
    if args.rate <= 0.0:
        print("--rate must be > 0", file=sys.stderr)
        return 2
    if args.rot_step_deg <= 0.0:
        print("--rot-step-deg must be > 0", file=sys.stderr)
        return 2
    if args.linear_speed <= 0.0:
        print("--linear-speed must be > 0", file=sys.stderr)
        return 2
    if args.angular_speed_deg <= 0.0:
        print("--angular-speed-deg must be > 0", file=sys.stderr)
        return 2
    if args.control_rate <= 0.0:
        print("--control-rate must be > 0", file=sys.stderr)
        return 2
    if args.base_speed <= 0.0:
        print("--base-speed must be > 0", file=sys.stderr)
        return 2
    if args.base_yaw_rate <= 0.0:
        print("--base-yaw-rate must be > 0", file=sys.stderr)
        return 2
    if args.recover_duration <= 0.0:
        print("--recover-duration must be > 0", file=sys.stderr)
        return 2
    if args.recover_rate <= 0.0:
        print("--recover-rate must be > 0", file=sys.stderr)
        return 2
    if args.recover_enter_wait < 0.0:
        print("--recover-enter-wait must be >= 0", file=sys.stderr)
        return 2
    if args.recover_settle_wait < 0.0:
        print("--recover-settle-wait must be >= 0", file=sys.stderr)
        return 2
    if args.recover_state_timeout < 0.0:
        print("--recover-state-timeout must be >= 0", file=sys.stderr)
        return 2
    if args.gripper_command_repeats <= 0:
        print("--gripper-command-repeats must be > 0", file=sys.stderr)
        return 2
    if args.gripper_repeat_interval < 0.0:
        print("--gripper-repeat-interval must be >= 0", file=sys.stderr)
        return 2
    if args.ocs2_cost_explosion_threshold <= 0.0:
        print("--ocs2-cost-explosion-threshold must be > 0", file=sys.stderr)
        return 2
    if args.max_position_error is not None and args.max_position_error <= 0.0:
        print("--max-position-error must be > 0", file=sys.stderr)
        return 2
    if args.hard_position_error is not None and args.hard_position_error <= 0.0:
        print("--hard-position-error must be > 0", file=sys.stderr)
        return 2
    if (
        args.hard_position_error is not None
        and args.max_position_error is not None
        and args.hard_position_error <= args.max_position_error
    ):
        print("--hard-position-error must be greater than --max-position-error", file=sys.stderr)
        return 2
    if args.left_max_target_offset is not None and args.left_max_axis_offset is None:
        print("--left-max-target-offset requires --left-max-axis-offset", file=sys.stderr)
        return 2
    if args.left_max_axis_offset is not None and args.left_max_target_offset is None:
        print("--left-max-axis-offset requires --left-max-target-offset", file=sys.stderr)
        return 2
    if args.right_max_target_offset is not None and args.right_max_axis_offset is None:
        print("--right-max-target-offset requires --right-max-axis-offset", file=sys.stderr)
        return 2
    if args.right_max_axis_offset is not None and args.right_max_target_offset is None:
        print("--right-max-axis-offset requires --right-max-target-offset", file=sys.stderr)
        return 2
    offsets = [v for v in (args.left_max_target_offset, args.right_max_target_offset) if v is not None]
    if any(v <= 0.0 for v in offsets):
        print("--left/right-max-target-offset must be > 0", file=sys.stderr)
        return 2
    axis_values = []
    if args.left_max_axis_offset is not None:
        axis_values.extend(args.left_max_axis_offset)
    if args.right_max_axis_offset is not None:
        axis_values.extend(args.right_max_axis_offset)
    if any(v <= 0.0 for v in axis_values):
        print("--left/right-max-axis-offset values must be > 0", file=sys.stderr)
        return 2
    if args.max_orientation_error_deg is not None and args.max_orientation_error_deg <= 0.0:
        print("--max-orientation-error-deg must be > 0", file=sys.stderr)
        return 2
    if args.error_gate_epsilon < 0.0:
        print("--error-gate-epsilon must be >= 0", file=sys.stderr)
        return 2
    if args.gate_print_period < 0.0:
        print("--gate-print-period must be >= 0", file=sys.stderr)
        return 2
    if args.translation_advice_axis_threshold < 0.0:
        print("--translation-advice-axis-threshold must be >= 0", file=sys.stderr)
        return 2
    if args.workspace_hit_epsilon < 0.0:
        print("--workspace-hit-epsilon must be >= 0", file=sys.stderr)
        return 2
    watchdog_position_values = [
        v for v in (args.watchdog_position_error, args.watchdog_severe_position_error) if v is not None
    ]
    if any(v <= 0.0 for v in watchdog_position_values):
        print("--watchdog position thresholds must be > 0", file=sys.stderr)
        return 2
    if (
        args.watchdog_position_error is not None
        and args.watchdog_severe_position_error is not None
        and args.watchdog_severe_position_error <= args.watchdog_position_error
    ):
        print("--watchdog-severe-position-error must be greater than --watchdog-position-error", file=sys.stderr)
        return 2
    watchdog_orientation_values = [
        v
        for v in (args.watchdog_orientation_error_deg, args.watchdog_severe_orientation_error_deg)
        if v is not None
    ]
    if any(v <= 0.0 for v in watchdog_orientation_values):
        print("--watchdog orientation thresholds must be > 0", file=sys.stderr)
        return 2
    if (
        args.watchdog_orientation_error_deg is not None
        and args.watchdog_severe_orientation_error_deg is not None
        and args.watchdog_severe_orientation_error_deg <= args.watchdog_orientation_error_deg
    ):
        print("--watchdog-severe-orientation-error-deg must be greater than --watchdog-orientation-error-deg", file=sys.stderr)
        return 2
    if args.right_joint1_lower_safety is not None and args.right_joint1_upper_safety is None:
        print("--right-joint1-lower-safety requires --right-joint1-upper-safety", file=sys.stderr)
        return 2
    if args.right_joint1_upper_safety is not None and args.right_joint1_lower_safety is None:
        print("--right-joint1-upper-safety requires --right-joint1-lower-safety", file=sys.stderr)
        return 2
    if (
        args.right_joint1_lower_safety is not None
        and args.right_joint1_upper_safety is not None
        and args.right_joint1_lower_safety >= args.right_joint1_upper_safety
    ):
        print("--right-joint1-lower-safety must be less than --right-joint1-upper-safety", file=sys.stderr)
        return 2
    if args.watchdog_print_period < 0.0:
        print("--watchdog-print-period must be >= 0", file=sys.stderr)
        return 2
    if args.camera_depth_save_every <= 0:
        print("--camera-depth-save-every must be > 0", file=sys.stderr)
        return 2
    if args.camera_stop_timeout <= 0.0:
        print("--camera-stop-timeout must be > 0", file=sys.stderr)
        return 2
    if args.camera_startup_grace < 0.0:
        print("--camera-startup-grace must be >= 0", file=sys.stderr)
        return 2
    if args.diagnostics_csv is None:
        args.diagnostics_csv = args.log_csv.with_suffix(".diagnostics.csv")
    if args.ocs2_log_dir is None:
        args.ocs2_log_dir = default_ocs2_log_dir()
    script_dir = Path(__file__).resolve().parent
    args.log_csv = args.log_csv.expanduser().resolve()
    args.camera_recorder_script = script_dir / "tools" / "camera_recorder.py"
    args.camera_output_dir = args.log_csv.parent / args.log_csv.stem / "camera"

    ensure_ros_log_dir()
    rclpy.init(args=None)
    node = KeyboardOcs2Teleop(args)
    try:
        print("Waiting for current poses from OCS2...")
        if not node.wait_for_current_poses(args.wait_timeout):
            print(
                "Timed out waiting for /left_current_pose and /right_current_pose. "
                "Start Isaac Sim Play and ocs2_isaac.launch.py first.",
                file=sys.stderr,
            )
            return 1

        if args.reset_targets:
            node.reset_targets_to_current()
        else:
            node.publish_targets()
            node.refresh_workspace_center()

        if not args.no_activate_ocs2:
            node.activate_ocs2_mode()

        node.camera_recorder.start()
        print(HELP)
        print(f"log: {args.log_csv.resolve()}")
        print(f"diagnostics: {args.diagnostics_csv.resolve()}")
        if args.record_cameras:
            print(f"camera: {args.camera_output_dir.resolve()}")
        print(f"ocs2 log scan dir: {args.ocs2_log_dir}")
        if args.input_mode == "pygame":
            status = node._run_pygame_loop()
            if status != 0:
                return status
        else:
            node._run_terminal_loop()
    except KeyboardInterrupt:
        node.log_row("keyboard_interrupt")
    finally:
        # rclpy's SIGINT handler may already have shut down the context. Always
        # finalize the independent recorder before any ROS operation can raise.
        node.camera_recorder.stop()
        if args.enable_base_motion and rclpy.ok():
            for _ in range(10):
                node.publish_base_stop("base_stop")
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(0.02)
        if not args.no_reset_on_exit and rclpy.ok():
            node.reset_after_task()
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
