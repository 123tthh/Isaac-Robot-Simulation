#!/usr/bin/env python3
"""
ROS2 6-camera grid viewer for SIM1 teach mode.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.md
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/sensor_msgs/msg/Image.md
  - /home/gtk/Trajectory/SIM1/tools/camera_recorder.py
  - /home/gtk/Trajectory/SIM1/graph/ALL_Graph.md
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Sequence

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from camera_recorder import DEFAULT_CAMERAS, image_to_depth_numpy, image_to_rgb_numpy


DISPLAY_ORDER = [
    "head_rgb",
    "left_rgb",
    "right_rgb",
    "head_depth",
    "left_depth",
    "right_depth",
]

DISPLAY_TITLES = {
    "head_rgb": "HEAD RGB",
    "left_rgb": "LEFT RGB",
    "right_rgb": "RIGHT RGB",
    "head_depth": "HEAD DEPTH",
    "left_depth": "LEFT DEPTH",
    "right_depth": "RIGHT DEPTH",
}


def ensure_ros_log_dir() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    ros_home = Path(os.environ.get("ROS_HOME", str(repo_root / ".ros")))
    log_dir = ros_home / "log"
    log_dir.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("ROS_HOME", str(ros_home))
    os.environ.setdefault("ROS_LOG_DIR", str(log_dir))


def ros_time_to_float(msg: Image) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def fit_image_to_cell(image: np.ndarray, cell_width: int, cell_height: int) -> np.ndarray:
    src_height, src_width = image.shape[:2]
    if src_height <= 0 or src_width <= 0:
        return np.zeros((cell_height, cell_width, 3), dtype=np.uint8)

    scale = min(cell_width / src_width, cell_height / src_height)
    target_width = max(1, int(round(src_width * scale)))
    target_height = max(1, int(round(src_height * scale)))
    resized = cv2.resize(image, (target_width, target_height), interpolation=cv2.INTER_AREA)

    canvas = np.zeros((cell_height, cell_width, 3), dtype=np.uint8)
    offset_x = (cell_width - target_width) // 2
    offset_y = (cell_height - target_height) // 2
    canvas[offset_y:offset_y + target_height, offset_x:offset_x + target_width] = resized
    return canvas


def depth_to_colormap(depth: np.ndarray, percentile: float) -> np.ndarray:
    depth_float = depth.astype(np.float32, copy=False)
    valid = np.isfinite(depth_float) & (depth_float > 0.0)

    if not np.any(valid):
        return np.zeros((depth.shape[0], depth.shape[1], 3), dtype=np.uint8)

    valid_values = depth_float[valid]
    low = float(np.min(valid_values))
    high = float(np.percentile(valid_values, percentile))
    if high <= low:
        high = low + 1e-6

    normalized = np.zeros_like(depth_float, dtype=np.float32)
    normalized[valid] = np.clip((depth_float[valid] - low) / (high - low), 0.0, 1.0)
    gray = (255.0 * (1.0 - normalized)).astype(np.uint8)
    color = cv2.applyColorMap(gray, cv2.COLORMAP_TURBO)
    color[~valid] = 0
    return color


@dataclass
class FrameState:
    image_bgr: Optional[np.ndarray] = None
    encoding: str = ""
    frame_id: str = ""
    ros_time_sec: Optional[float] = None
    last_wall_time_sec: Optional[float] = None
    frame_count: int = 0
    publisher_count: int = 0
    status: str = "waiting"


class CameraGridViewer(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("sim1_camera_grid_viewer")
        self.args = args
        self.frame_states: Dict[str, FrameState] = {name: FrameState() for name in DISPLAY_ORDER}
        self._image_subscriptions = []
        self.last_error_log: Dict[str, float] = {}
        self.last_graph_check_time = 0.0

        qos = self.make_qos(args.reliability)
        for name in DISPLAY_ORDER:
            topic = DEFAULT_CAMERAS[name]["topic"]
            sub = self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self.on_image(msg, camera_name),
                qos,
            )
            self._image_subscriptions.append(sub)
            self.get_logger().info(f"Subscribed {name}: {topic}")

    def update_graph_status(self) -> None:
        now = time.time()
        if now - self.last_graph_check_time < 1.0:
            return

        for name in DISPLAY_ORDER:
            topic = DEFAULT_CAMERAS[name]["topic"]
            self.frame_states[name].publisher_count = self.count_publishers(topic)
        self.last_graph_check_time = now

    @staticmethod
    def make_qos(reliability: str) -> QoSProfile:
        if reliability == "best_effort":
            rel = ReliabilityPolicy.BEST_EFFORT
        else:
            rel = ReliabilityPolicy.RELIABLE

        return QoSProfile(
            reliability=rel,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def on_image(self, msg: Image, name: str) -> None:
        state = self.frame_states[name]

        try:
            if name.endswith("_rgb"):
                rgb = image_to_rgb_numpy(msg)
                image_bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            else:
                depth = image_to_depth_numpy(msg)
                image_bgr = depth_to_colormap(depth, self.args.depth_percentile)
        except Exception as exc:  # pragma: no cover - runtime topic dependent
            now = time.time()
            if now - self.last_error_log.get(name, 0.0) > 2.0:
                self.get_logger().warning(f"{name}: failed to decode frame: {exc}")
                self.last_error_log[name] = now
            state.status = f"decode error: {msg.encoding}"
            return

        state.image_bgr = image_bgr
        state.encoding = msg.encoding
        state.frame_id = msg.header.frame_id
        state.ros_time_sec = ros_time_to_float(msg)
        state.last_wall_time_sec = time.time()
        state.frame_count += 1
        state.status = "ok"

    def render_cell(self, name: str) -> np.ndarray:
        cell = np.zeros((self.args.cell_height, self.args.cell_width, 3), dtype=np.uint8)
        cell[:] = (18, 18, 18)
        state = self.frame_states[name]

        if state.image_bgr is not None:
            cell = fit_image_to_cell(state.image_bgr, self.args.cell_width, self.args.cell_height)

        overlay = cell.copy()
        cv2.rectangle(overlay, (0, 0), (self.args.cell_width, 34), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.55, cell, 0.45, 0.0, cell)

        title = DISPLAY_TITLES[name]
        topic = DEFAULT_CAMERAS[name]["topic"]
        age_text = "age=--"
        if state.last_wall_time_sec is not None:
            age_ms = max(0.0, (time.time() - state.last_wall_time_sec) * 1000.0)
            age_text = f"age={age_ms:.0f}ms"

        footer_lines = [
            title,
            topic,
            f"pub={state.publisher_count}  {state.encoding or 'no-frame'}  frames={state.frame_count}  {age_text}",
        ]

        if state.image_bgr is None:
            footer_lines.append("no publisher" if state.publisher_count == 0 else state.status)

        text_y = 16
        for idx, line in enumerate(footer_lines):
            scale = 0.50 if idx == 0 else 0.42
            thickness = 1 if idx > 0 else 2
            color = (255, 255, 255) if idx == 0 else (210, 210, 210)
            cv2.putText(
                cell,
                line,
                (8, text_y),
                cv2.FONT_HERSHEY_SIMPLEX,
                scale,
                color,
                thickness,
                cv2.LINE_AA,
            )
            text_y += 16

        return cell

    def render_canvas(self) -> np.ndarray:
        self.update_graph_status()
        cells = [self.render_cell(name) for name in DISPLAY_ORDER]
        top_row = np.hstack(cells[:3])
        bottom_row = np.hstack(cells[3:])
        canvas = np.vstack([top_row, bottom_row])

        cv2.putText(
            canvas,
            "Press Q or Esc to close",
            (12, canvas.shape[0] - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        return canvas


def parse_args(argv: Sequence[str]) -> tuple[argparse.Namespace, Sequence[str]]:
    parser = argparse.ArgumentParser(description="Show 6 ROS camera topics in one 2x3 grid window.")
    parser.add_argument("--cell-width", type=int, default=480, help="Display width of each grid cell.")
    parser.add_argument("--cell-height", type=int, default=270, help="Display height of each grid cell.")
    parser.add_argument(
        "--depth-percentile",
        type=float,
        default=98.0,
        help="Upper percentile used for depth pseudo-color normalization.",
    )
    parser.add_argument(
        "--display-fps",
        type=float,
        default=30.0,
        help="Maximum UI refresh rate.",
    )
    parser.add_argument(
        "--reliability",
        choices=["reliable", "best_effort"],
        default="best_effort",
        help="ROS2 QoS reliability (best_effort matches Isaac Sim sensor-data publishers).",
    )
    parser.add_argument(
        "--window-title",
        default="SIM1 Camera Grid",
        help="OpenCV window title.",
    )
    return parser.parse_known_args(argv)


def main(argv: Optional[Sequence[str]] = None) -> int:
    ensure_ros_log_dir()
    args, ros_args = parse_args(sys.argv[1:] if argv is None else argv)

    rclpy.init(args=list(ros_args))
    node = CameraGridViewer(args)

    cv2.namedWindow(args.window_title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(args.window_title, args.cell_width * 3, args.cell_height * 2)

    frame_interval = 1.0 / max(args.display_fps, 1.0)
    next_frame_time = 0.0

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            now = time.time()
            if now < next_frame_time:
                continue

            canvas = node.render_canvas()
            cv2.imshow(args.window_title, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (27, ord("q"), ord("Q")):
                break
            next_frame_time = now + frame_interval
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
