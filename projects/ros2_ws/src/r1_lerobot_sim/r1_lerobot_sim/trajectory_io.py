# Reference: /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.md
# Reference: /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.md

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


ARM_JOINT_NAMES = [
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

LEROBOT_STATE_NAMES = [
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
    "left_gripper",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
    "right_gripper",
]

POSE_FIELD_NAMES = [
    "left_x",
    "left_y",
    "left_z",
    "left_qx",
    "left_qy",
    "left_qz",
    "left_qw",
    "left_gripper",
    "right_x",
    "right_y",
    "right_z",
    "right_qx",
    "right_qy",
    "right_qz",
    "right_qw",
    "right_gripper",
]


@dataclass(frozen=True)
class Trajectory:
    timestamps: np.ndarray
    joint_names: list[str]
    positions: np.ndarray
    gripper: np.ndarray | None = None


@dataclass(frozen=True)
class PoseTrajectory:
    timestamps: np.ndarray
    poses: np.ndarray


def read_urdf_movable_joints(urdf_path: str | Path) -> tuple[list[str], dict[str, float]]:
    root = ET.parse(str(urdf_path)).getroot()
    names: list[str] = []
    defaults: dict[str, float] = {}
    for joint in root.findall("joint"):
        name = joint.get("name")
        joint_type = joint.get("type")
        if not name or joint_type == "fixed":
            continue
        names.append(name)
        limit = joint.find("limit")
        if limit is not None and limit.get("lower") is not None and limit.get("upper") is not None:
            lower = float(limit.get("lower"))
            upper = float(limit.get("upper"))
            defaults[name] = 0.5 * (lower + upper)
        else:
            defaults[name] = 0.0
    return names, defaults


def load_joint_trajectory(
    dataset_path: str | Path,
    episode_index: int = 0,
    field: str = "action",
    npz_path: str | Path | None = None,
) -> Trajectory:
    if npz_path:
        return _load_npz_joint_trajectory(npz_path)
    dataset = Path(dataset_path).expanduser()
    export_npz = dataset / f"episode_{episode_index:03d}_rm_dual_joint_traj.npz"
    if export_npz.exists() and field in ("action", "observation.state"):
        return _load_npz_joint_trajectory(export_npz)
    return _load_lerobot_joint_trajectory(dataset, episode_index, field)


def load_pose_trajectory(
    dataset_path: str | Path,
    episode_index: int = 0,
    field: str = "action.pose",
) -> PoseTrajectory:
    data = _load_lerobot_rows(Path(dataset_path).expanduser(), episode_index, [field, "timestamp"])
    poses = np.asarray(data[field], dtype=np.float64)
    timestamps = np.asarray(data["timestamp"], dtype=np.float64)
    return PoseTrajectory(timestamps=timestamps, poses=poses)


def _load_npz_joint_trajectory(npz_path: str | Path) -> Trajectory:
    data = np.load(str(npz_path), allow_pickle=True)
    return Trajectory(
        timestamps=np.asarray(data["t"], dtype=np.float64),
        joint_names=[str(name) for name in data["joint_names"].tolist()],
        positions=np.asarray(data["q"], dtype=np.float64),
        gripper=np.asarray(data["gripper"], dtype=np.float64) if "gripper" in data.files else None,
    )


def _load_lerobot_joint_trajectory(dataset: Path, episode_index: int, field: str) -> Trajectory:
    data = _load_lerobot_rows(dataset, episode_index, [field, "timestamp"])
    raw = np.asarray(data[field], dtype=np.float64)
    timestamps = np.asarray(data["timestamp"], dtype=np.float64)
    name_to_index = {name: idx for idx, name in enumerate(LEROBOT_STATE_NAMES)}
    positions = np.asarray([[row[name_to_index[name]] for name in ARM_JOINT_NAMES] for row in raw])
    gripper = np.asarray(
        [[row[name_to_index["left_gripper"]], row[name_to_index["right_gripper"]]] for row in raw],
        dtype=np.float64,
    )
    return Trajectory(timestamps=timestamps, joint_names=list(ARM_JOINT_NAMES), positions=positions, gripper=gripper)


def _load_lerobot_rows(dataset: Path, episode_index: int, columns: list[str]) -> dict[str, list]:
    import pyarrow.parquet as pq

    info = json.loads((dataset / "meta" / "info.json").read_text())
    data_template = info["data_path"]
    episodes_path = dataset / "meta" / "episodes" / "chunk-000" / "file-000.parquet"
    episodes = pq.read_table(str(episodes_path)).to_pydict()
    try:
        row_index = episodes["episode_index"].index(episode_index)
    except ValueError as exc:
        raise ValueError(f"episode_index={episode_index} not found in {episodes_path}") from exc
    chunk_index = int(episodes["data/chunk_index"][row_index])
    file_index = int(episodes["data/file_index"][row_index])
    start = int(episodes["dataset_from_index"][row_index])
    stop = int(episodes["dataset_to_index"][row_index])
    parquet_path = dataset / data_template.format(chunk_index=chunk_index, file_index=file_index)
    table = pq.read_table(str(parquet_path), columns=columns + ["episode_index"])
    rows = table.to_pydict()
    mask = [idx for idx, ep in enumerate(rows["episode_index"]) if int(ep) == episode_index]
    if not mask:
        mask = list(range(start, stop))
    return {col: [rows[col][idx] for idx in mask] for col in columns}


def map_gripper_to_pgia(
    value: float,
    mode: str,
    closed: float,
    open_: float,
    source_min: float | None = None,
    source_max: float | None = None,
) -> float:
    if mode == "meters":
        return float(value)
    if mode == "episode_minmax":
        if source_min is None or source_max is None or abs(source_max - source_min) < 1.0e-9:
            alpha = 0.0
        else:
            alpha = (float(value) - source_min) / (source_max - source_min)
    if mode == "percent_0_100":
        alpha = max(0.0, min(1.0, float(value) / 100.0))
    elif mode != "episode_minmax":
        alpha = max(0.0, min(1.0, float(value)))
    return closed + alpha * (open_ - closed)
