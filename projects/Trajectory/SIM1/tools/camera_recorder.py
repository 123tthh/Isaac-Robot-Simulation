#!/usr/bin/env python3
"""
Standalone ROS2 camera recorder for Isaac Sim / SIM1.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/rclpy.node.md
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/rclpy.executors.md
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/examples_rclpy_minimal_subscriber/examples_rclpy_minimal_subscriber.subscriber_member_function.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md

Records:
  RGB topics   -> mp4
  Depth topics -> .npy frame files
  timestamps   -> camera_timestamps.csv
  summary      -> summary.json

Default topics:
  /head_cam/color/image_raw
  /head_cam/depth/image_rect_raw
  /left_cam/color/image_raw
  /left_cam/depth/image_rect_raw
  /right_cam/color/image_raw
  /right_cam/depth/image_rect_raw

Usage:
  python3 tools/camera_recorder.py --duration 10

RGB-only test:
  python3 tools/camera_recorder.py --duration 10 --rgb-only

Output:
  trace_data/raw/camera_test_YYYYmmdd_HHMMSS/
"""

import argparse
import csv
import json
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


DEFAULT_CAMERAS = {
    "head_rgb": {
        "topic": "/head_cam/color/image_raw",
        "type": "rgb",
    },
    "head_depth": {
        "topic": "/head_cam/depth/image_rect_raw",
        "type": "depth",
    },
    "left_rgb": {
        "topic": "/left_cam/color/image_raw",
        "type": "rgb",
    },
    "left_depth": {
        "topic": "/left_cam/depth/image_rect_raw",
        "type": "depth",
    },
    "right_rgb": {
        "topic": "/right_cam/color/image_raw",
        "type": "rgb",
    },
    "right_depth": {
        "topic": "/right_cam/depth/image_rect_raw",
        "type": "depth",
    },
}


def ros_time_to_float(msg: Image) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


def image_to_rgb_numpy(msg: Image) -> np.ndarray:
    """
    Convert sensor_msgs/Image to RGB uint8 numpy array.

    Supported:
      rgb8
      bgr8
      rgba8
      bgra8
      mono8
    """
    h, w = msg.height, msg.width
    enc = msg.encoding.lower()

    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return arr.copy()

    if enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 3)
        return cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)

    if enc == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2RGB)

    if enc == "bgra8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w, 4)
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2RGB)

    if enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(h, w)
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2RGB)

    raise ValueError(f"Unsupported RGB encoding: {msg.encoding}")


def image_to_depth_numpy(msg: Image) -> np.ndarray:
    """
    Convert sensor_msgs/Image to depth numpy array.

    Supported:
      32FC1 -> float32, usually meters
      16UC1 -> uint16, often millimeters depending on publisher
    """
    h, w = msg.height, msg.width
    enc = msg.encoding.upper()

    if enc == "32FC1":
        return np.frombuffer(msg.data, dtype=np.float32).reshape(h, w).copy()

    if enc == "16UC1":
        return np.frombuffer(msg.data, dtype=np.uint16).reshape(h, w).copy()

    raise ValueError(f"Unsupported depth encoding: {msg.encoding}")


class CameraStreamState:
    def __init__(self, name: str, cfg: dict, output_dir: Path, rgb_fps: float):
        self.name = name
        self.topic = cfg["topic"]
        self.stream_type = cfg["type"]
        self.output_dir = output_dir
        self.rgb_fps = rgb_fps

        self.frame_count = 0
        self.first_ros_time: Optional[float] = None
        self.last_ros_time: Optional[float] = None
        self.first_wall_time: Optional[float] = None
        self.last_wall_time: Optional[float] = None

        self.width: Optional[int] = None
        self.height: Optional[int] = None
        self.encoding: Optional[str] = None
        self.frame_id: Optional[str] = None

        self.video_writer: Optional[cv2.VideoWriter] = None

        if self.stream_type == "rgb":
            self.stream_dir = output_dir / "videos"
            self.stream_dir.mkdir(parents=True, exist_ok=True)
            self.video_path = self.stream_dir / f"{name}.mp4"
        else:
            self.stream_dir = output_dir / "depth" / name
            self.stream_dir.mkdir(parents=True, exist_ok=True)
            self.video_path = None

    def update_meta(self, msg: Image):
        ros_t = ros_time_to_float(msg)
        wall_t = time.time()

        if self.first_ros_time is None:
            self.first_ros_time = ros_t
            self.first_wall_time = wall_t

        self.last_ros_time = ros_t
        self.last_wall_time = wall_t

        self.width = msg.width
        self.height = msg.height
        self.encoding = msg.encoding
        self.frame_id = msg.header.frame_id

    def ensure_video_writer(self, rgb: np.ndarray):
        if self.video_writer is not None:
            return

        h, w = rgb.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        self.video_writer = cv2.VideoWriter(
            str(self.video_path),
            fourcc,
            self.rgb_fps,
            (w, h),
        )

        if not self.video_writer.isOpened():
            raise RuntimeError(f"Failed to open video writer: {self.video_path}")

    def close(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None

    def summary(self):
        duration_ros = None
        duration_wall = None
        fps_ros = None
        fps_wall = None

        if self.first_ros_time is not None and self.last_ros_time is not None:
            duration_ros = max(0.0, self.last_ros_time - self.first_ros_time)
            if duration_ros > 1e-6:
                fps_ros = self.frame_count / duration_ros

        if self.first_wall_time is not None and self.last_wall_time is not None:
            duration_wall = max(0.0, self.last_wall_time - self.first_wall_time)
            if duration_wall > 1e-6:
                fps_wall = self.frame_count / duration_wall

        return {
            "name": self.name,
            "topic": self.topic,
            "type": self.stream_type,
            "frame_count": self.frame_count,
            "width": self.width,
            "height": self.height,
            "encoding": self.encoding,
            "frame_id": self.frame_id,
            "duration_ros_sec": duration_ros,
            "duration_wall_sec": duration_wall,
            "fps_ros_est": fps_ros,
            "fps_wall_est": fps_wall,
            "output": str(self.video_path if self.stream_type == "rgb" else self.stream_dir),
        }


class CameraRecorder(Node):
    def __init__(
        self,
        output_dir: Path,
        cameras: Dict[str, dict],
        rgb_fps: float,
        depth_clip_max: Optional[float],
        depth_save_every: int,
        reliability: str,
    ):
        super().__init__("camera_recorder")

        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.rgb_fps = rgb_fps
        self.depth_clip_max = depth_clip_max
        self.depth_save_every = max(1, depth_save_every)

        self.states: Dict[str, CameraStreamState] = {}
        self.subs = []
        self.running = True

        self.csv_path = self.output_dir / "camera_timestamps.csv"
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.DictWriter(
            self.csv_file,
            fieldnames=[
                "camera_name",
                "topic",
                "stream_type",
                "frame_index",
                "ros_time_sec",
                "wall_time_sec",
                "frame_id",
                "encoding",
                "width",
                "height",
                "relative_path",
            ],
        )
        self.csv_writer.writeheader()

        qos = self.make_qos(reliability)

        for name, cfg in cameras.items():
            state = CameraStreamState(name, cfg, self.output_dir, rgb_fps)
            self.states[name] = state

            sub = self.create_subscription(
                Image,
                cfg["topic"],
                lambda msg, n=name: self.on_image(msg, n),
                qos,
            )
            self.subs.append(sub)
            self.get_logger().info(f"Subscribed {name}: {cfg['topic']}")

    @staticmethod
    def make_qos(reliability: str) -> QoSProfile:
        if reliability == "reliable":
            rel = ReliabilityPolicy.RELIABLE
        elif reliability == "best_effort":
            rel = ReliabilityPolicy.BEST_EFFORT
        else:
            # Isaac Sim sensor/camera publishers use the ROS 2 sensor-data QoS.
            # Keep the fallback compatible with BEST_EFFORT image publishers.
            rel = ReliabilityPolicy.BEST_EFFORT

        return QoSProfile(
            reliability=rel,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def on_image(self, msg: Image, name: str):
        state = self.states[name]
        state.update_meta(msg)

        frame_idx = state.frame_count
        ros_t = ros_time_to_float(msg)
        wall_t = time.time()

        try:
            if state.stream_type == "rgb":
                rgb = image_to_rgb_numpy(msg)
                state.ensure_video_writer(rgb)

                # OpenCV VideoWriter expects BGR.
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                state.video_writer.write(bgr)

                rel_path = str(Path("videos") / f"{name}.mp4")

            else:
                if frame_idx % self.depth_save_every != 0:
                    self.csv_writer.writerow(
                        {
                            "camera_name": name,
                            "topic": state.topic,
                            "stream_type": state.stream_type,
                            "frame_index": frame_idx,
                            "ros_time_sec": f"{ros_t:.9f}",
                            "wall_time_sec": f"{wall_t:.9f}",
                            "frame_id": msg.header.frame_id,
                            "encoding": msg.encoding,
                            "width": msg.width,
                            "height": msg.height,
                            "relative_path": "",
                        }
                    )
                    state.frame_count += 1
                    return

                depth = image_to_depth_numpy(msg)

                if self.depth_clip_max is not None and depth.dtype == np.float32:
                    depth = depth.copy()
                    invalid = ~np.isfinite(depth)
                    depth[invalid] = 0.0
                    depth[depth < 0.0] = 0.0
                    depth[depth > self.depth_clip_max] = self.depth_clip_max

                filename = f"{frame_idx:06d}.npy"
                path = state.stream_dir / filename
                np.save(path, depth)

                rel_path = str(Path("depth") / name / filename)

            self.csv_writer.writerow(
                {
                    "camera_name": name,
                    "topic": state.topic,
                    "stream_type": state.stream_type,
                    "frame_index": frame_idx,
                    "ros_time_sec": f"{ros_t:.9f}",
                    "wall_time_sec": f"{wall_t:.9f}",
                    "frame_id": msg.header.frame_id,
                    "encoding": msg.encoding,
                    "width": msg.width,
                    "height": msg.height,
                    "relative_path": rel_path,
                }
            )

            state.frame_count += 1

        except Exception as e:
            self.get_logger().error(f"[{name}] failed to record frame {frame_idx}: {e}")

    def close(self):
        for state in self.states.values():
            state.close()

        self.csv_file.flush()
        self.csv_file.close()

        summary = {
            "output_dir": str(self.output_dir),
            "created_at": datetime.now().isoformat(),
            "streams": {name: state.summary() for name, state in self.states.items()},
        }

        with open(self.output_dir / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)

        self.get_logger().info(f"Saved summary: {self.output_dir / 'summary.json'}")


def parse_args():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--output-root",
        type=str,
        default="trace_data/raw",
        help="Root output directory.",
    )

    parser.add_argument(
        "--name",
        type=str,
        default=None,
        help="Output run name. Default: camera_test_YYYYmmdd_HHMMSS",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Recording duration in seconds. 0 means record until Ctrl-C.",
    )

    parser.add_argument(
        "--rgb-only",
        action="store_true",
        help="Record only RGB topics.",
    )

    parser.add_argument(
        "--head-only",
        action="store_true",
        help="Record only head RGB/Depth.",
    )

    parser.add_argument(
        "--left-only",
        action="store_true",
        help="Record only left RGB/Depth.",
    )

    parser.add_argument(
        "--right-only",
        action="store_true",
        help="Record only right RGB/Depth.",
    )

    parser.add_argument(
        "--rgb-fps",
        type=float,
        default=15.0,
        help="FPS used for mp4 writing. Does not control ROS topic FPS.",
    )

    parser.add_argument(
        "--depth-clip-max",
        type=float,
        default=3.0,
        help="Clip float32 depth to this max value before saving. Use <=0 to disable.",
    )

    parser.add_argument(
        "--depth-save-every",
        type=int,
        default=1,
        help="Save every Nth depth frame.",
    )

    parser.add_argument(
        "--reliability",
        choices=["reliable", "best_effort"],
        default="best_effort",
        help="ROS2 QoS reliability (best_effort matches Isaac Sim sensor-data publishers).",
    )

    return parser.parse_args()


def select_cameras(args) -> Dict[str, dict]:
    cams = dict(DEFAULT_CAMERAS)

    if args.rgb_only:
        cams = {k: v for k, v in cams.items() if v["type"] == "rgb"}

    selected_side = None
    if args.head_only:
        selected_side = "head"
    if args.left_only:
        selected_side = "left"
    if args.right_only:
        selected_side = "right"

    if selected_side is not None:
        cams = {k: v for k, v in cams.items() if k.startswith(selected_side + "_")}

    return cams


def main():
    args = parse_args()

    if args.depth_clip_max <= 0:
        args.depth_clip_max = None

    run_name = args.name
    if run_name is None:
        run_name = "camera_test_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    output_dir = Path(args.output_root) / run_name
    cameras = select_cameras(args)

    if not cameras:
        print("[ERROR] no cameras selected.")
        return 1

    print("Recording cameras:")
    for name, cfg in cameras.items():
        print(f"  {name}: {cfg['topic']}")

    print("Output:", output_dir)

    rclpy.init()

    node = CameraRecorder(
        output_dir=output_dir,
        cameras=cameras,
        rgb_fps=args.rgb_fps,
        depth_clip_max=args.depth_clip_max,
        depth_save_every=args.depth_save_every,
        reliability=args.reliability,
    )

    stop_requested = {"flag": False}

    def handle_sigint(signum, frame):
        stop_requested["flag"] = True
        node.get_logger().info("Stop requested...")

    signal.signal(signal.SIGINT, handle_sigint)

    start = time.time()

    try:
        while rclpy.ok() and not stop_requested["flag"]:
            rclpy.spin_once(node, timeout_sec=0.05)

            if args.duration > 0 and (time.time() - start) >= args.duration:
                node.get_logger().info("Duration reached.")
                break

    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()

    print("\n[DONE] camera recording finished.")
    print("Output:", output_dir)
    print("Summary:", output_dir / "summary.json")
    print("Timestamps:", output_dir / "camera_timestamps.csv")
    return 0


if __name__ == "__main__":
    sys.exit(main())
