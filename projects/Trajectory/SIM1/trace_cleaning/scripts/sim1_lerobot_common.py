#!/usr/bin/env python3
"""
Shared SIM1 trace-to-LeRobot helpers.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/rclpy.node.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md

External schema references:
  - https://huggingface.co/docs/lerobot/lerobot-dataset-v3
  - https://huggingface.co/docs/lerobot/main/porting_datasets_v3
"""

from __future__ import annotations

import csv
import json
import math
import shutil
from pathlib import Path
from typing import Iterable

import numpy as np
import pandas as pd


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

STATE_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
    "left_gripper_closed",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
    "right_gripper_closed",
]

POSE_NAMES = [
    "left_x",
    "left_y",
    "left_z",
    "left_qx",
    "left_qy",
    "left_qz",
    "left_qw",
    "left_gripper_closed",
    "right_x",
    "right_y",
    "right_z",
    "right_qx",
    "right_qy",
    "right_qz",
    "right_qw",
    "right_gripper_closed",
]

CAMERA_VIDEO_KEYS = {
    "head_rgb": "observation.images.head",
    "left_rgb": "observation.images.left_wrist",
    "right_rgb": "observation.images.right_wrist",
}


def parse_float_list(text: str) -> list[float]:
    text = (text or "").strip()
    if not text:
        return []
    return [float(part) for part in text.replace(",", " ").split()]


def split_names(text: str) -> list[str]:
    return [part.strip() for part in (text or "").split() if part.strip()]


def parse_gripper_closed(row: dict[str, str], side: str, fallback: float | None) -> float:
    closed_col = f"{side}_gripper_closed"
    if row.get(closed_col, "").strip() != "":
        return float(row[closed_col])
    cmd = row.get(f"{side}_gripper_cmd", "").strip()
    if cmd and cmd.lower() != "nan":
        return 1.0 if float(cmd) <= 0.0 else 0.0
    if fallback is not None:
        return float(fallback)
    raise ValueError(f"missing {closed_col}; re-record with the updated teleop script or provide gripper commands")


def values_by_name(names_text: str, values_text: str, wanted: Iterable[str], label: str) -> list[float]:
    names = split_names(names_text)
    values = parse_float_list(values_text)
    if len(names) != len(values):
        raise ValueError(f"{label} has {len(names)} names but {len(values)} values")
    by_name = dict(zip(names, values, strict=True))
    missing = [name for name in wanted if name not in by_name]
    if missing:
        raise ValueError(f"{label} is missing required arm joints: {', '.join(missing)}")
    return [float(by_name[name]) for name in wanted]


def pose_vector(row: dict[str, str], left_closed: float, right_closed: float) -> list[float]:
    left_xyz = parse_float_list(row.get("left_target_xyz", ""))
    left_xyzw = parse_float_list(row.get("left_target_xyzw", ""))
    right_xyz = parse_float_list(row.get("right_target_xyz", ""))
    right_xyzw = parse_float_list(row.get("right_target_xyzw", ""))
    if len(left_xyz) != 3 or len(right_xyz) != 3 or len(left_xyzw) != 4 or len(right_xyzw) != 4:
        raise ValueError("target pose columns must contain xyz and xyzw values")
    return left_xyz + left_xyzw + [left_closed] + right_xyz + right_xyzw + [right_closed]


def next_or_self(values: list[list[float]]) -> list[list[float]]:
    if not values:
        return []
    return values[1:] + [values[-1]]


def stats_for_array(values: np.ndarray) -> dict[str, list[float] | list[int]]:
    if values.ndim == 1:
        values = values.reshape(-1, 1)
    return {
        "min": values.min(axis=0).astype(float).tolist(),
        "max": values.max(axis=0).astype(float).tolist(),
        "mean": values.mean(axis=0).astype(float).tolist(),
        "std": values.std(axis=0).astype(float).tolist(),
        "count": [int(values.shape[0])],
    }


def stats_for_column(series: pd.Series) -> dict[str, list[float] | list[int]]:
    first = series.iloc[0]
    if isinstance(first, (list, tuple, np.ndarray)):
        arr = np.asarray(series.to_list(), dtype=np.float32)
    else:
        arr = series.to_numpy()
    return stats_for_array(arr)


def feature(dtype: str, shape: list[int], names: list[str] | None, fps: int) -> dict[str, object]:
    return {"dtype": dtype, "shape": shape, "names": [",".join(names)] if names else None, "fps": fps}


def video_feature(shape: list[int], fps: int) -> dict[str, object]:
    return {"dtype": "video", "shape": shape, "names": ["height,width,channels"], "fps": fps}


def load_trace(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            raise ValueError(f"{path} has no CSV header")
        rows = [row for row in reader if row.get("event") not in {"quit", "keyboard_interrupt", "reset_after_task"}]
    if not rows:
        raise ValueError(f"{path} contains no usable rows")
    required = {"wall_time", "joint_names", "joint_positions", "left_target_xyz", "right_target_xyz"}
    missing = sorted(required - set(rows[0]))
    if missing:
        raise ValueError(f"{path} is missing required columns: {', '.join(missing)}")
    return rows


def camera_dir_for_trace(trace: Path, explicit: Path | None = None) -> Path | None:
    if explicit is not None:
        return explicit if explicit.exists() else None
    sibling = trace.with_suffix("") / "camera"
    if sibling.exists():
        return sibling
    direct = trace.with_suffix("")
    if (direct / "camera_timestamps.csv").exists():
        return direct
    return None


def load_camera_rows(camera_dir: Path | None) -> dict[str, list[dict[str, str]]]:
    if camera_dir is None:
        return {}
    path = camera_dir / "camera_timestamps.csv"
    if not path.exists():
        return {}
    by_camera: dict[str, list[dict[str, str]]] = {}
    with path.open(newline="", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            by_camera.setdefault(row["camera_name"], []).append(row)
    for rows in by_camera.values():
        rows.sort(key=lambda row: float(row["wall_time_sec"]))
    return by_camera


def nearest_camera_indices(trace_rows: list[dict[str, str]], camera_rows: list[dict[str, str]]) -> list[int]:
    if not camera_rows:
        return [-1] * len(trace_rows)
    camera_times = [float(row["wall_time_sec"]) for row in camera_rows]
    out = []
    j = 0
    for row in trace_rows:
        t = float(row["wall_time"])
        while j + 1 < len(camera_times) and abs(camera_times[j + 1] - t) <= abs(camera_times[j] - t):
            j += 1
        out.append(j)
    return out


def build_dataset_rows(
    rows: list[dict[str, str]],
    missing_gripper_closed: float | None,
    allow_missing_effort: bool,
    camera_rows_by_name: dict[str, list[dict[str, str]]] | None = None,
) -> tuple[pd.DataFrame, dict[str, object], dict[str, list[int]]]:
    states: list[list[float]] = []
    poses: list[list[float]] = []
    efforts: list[list[float]] = []
    source_wall0 = float(rows[0]["wall_time"])

    for row in rows:
        left_closed = parse_gripper_closed(row, "left", missing_gripper_closed)
        right_closed = parse_gripper_closed(row, "right", missing_gripper_closed)
        joint_pos = values_by_name(row["joint_names"], row["joint_positions"], ARM_JOINTS, "joint_positions")
        left_joints = joint_pos[:7]
        right_joints = joint_pos[7:]
        states.append(left_joints + [left_closed] + right_joints + [right_closed])
        poses.append(pose_vector(row, left_closed, right_closed))

        effort_names = row.get("effort_joint_names", "")
        effort_values = row.get("joint_efforts", "")
        if effort_names.strip() and effort_values.strip():
            efforts.append(values_by_name(effort_names, effort_values, ARM_JOINTS, "joint_efforts"))
        elif allow_missing_effort:
            efforts.append([0.0] * len(ARM_JOINTS))
        else:
            raise ValueError(
                "missing joint_efforts/effort_joint_names; re-record with the updated teleop script "
                "or pass --allow-missing-effort only for legacy compatibility checks"
            )

    timestamps = [max(float(row["wall_time"]) - source_wall0, 0.0) for row in rows]
    frame_count = len(rows)
    data = pd.DataFrame(
        {
            "observation.state": [np.asarray(v, dtype=np.float32) for v in states],
            "observation.state.pose": [np.asarray(v, dtype=np.float32) for v in poses],
            "observation.effort": [np.asarray(v, dtype=np.float32) for v in efforts],
            "action": [np.asarray(v, dtype=np.float32) for v in next_or_self(states)],
            "action.pose": [np.asarray(v, dtype=np.float32) for v in next_or_self(poses)],
            "timestamp": np.asarray(timestamps, dtype=np.float32),
            "frame_index": np.arange(frame_count, dtype=np.int64),
            "episode_index": np.zeros(frame_count, dtype=np.int64),
            "index": np.arange(frame_count, dtype=np.int64),
            "task_index": np.zeros(frame_count, dtype=np.int64),
        }
    )

    camera_indices: dict[str, list[int]] = {}
    camera_rows_by_name = camera_rows_by_name or {}
    for camera_name, video_key in CAMERA_VIDEO_KEYS.items():
        indices = nearest_camera_indices(rows, camera_rows_by_name.get(camera_name, []))
        if any(index >= 0 for index in indices):
            camera_indices[camera_name] = indices
            data[video_key] = [
                {"path": f"videos/{video_key}/chunk-000/file-000.mp4", "timestamp": timestamps[i]}
                for i in range(frame_count)
            ]

    stats = {
        column: stats_for_column(data[column])
        for column in data.columns
        if not column.startswith("observation.images.")
    }
    return data, stats, camera_indices


def camera_shape(camera_dir: Path | None, camera_name: str) -> list[int]:
    if camera_dir is not None:
        summary_path = camera_dir / "summary.json"
        if summary_path.exists():
            try:
                summary = json.loads(summary_path.read_text(encoding="utf-8"))
                stream = summary.get("streams", {}).get(camera_name, {})
                h = int(stream.get("height") or 480)
                w = int(stream.get("width") or 640)
                return [h, w, 3]
            except (OSError, json.JSONDecodeError, ValueError, TypeError):
                pass
    return [480, 640, 3]


def copy_selected_videos(
    camera_dir: Path | None,
    output_root: Path,
    camera_rows_by_name: dict[str, list[dict[str, str]]],
    camera_indices: dict[str, list[int]],
    v30: bool,
) -> None:
    if camera_dir is None:
        return
    for camera_name, video_key in CAMERA_VIDEO_KEYS.items():
        if camera_name not in camera_indices:
            continue
        rows = camera_rows_by_name.get(camera_name, [])
        if not rows:
            continue
        rel_path = rows[0].get("relative_path", "")
        if not rel_path:
            continue
        source = camera_dir / rel_path
        if not source.exists():
            continue
        if v30:
            dest = output_root / "videos" / video_key / "chunk-000" / "file-000.mp4"
        else:
            dest = output_root / "videos" / "chunk-000" / video_key / "episode_000000.mp4"
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, dest)
