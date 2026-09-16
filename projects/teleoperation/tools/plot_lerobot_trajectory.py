#!/usr/bin/env python3
# Reference: /home/gtk/ai_docs/docs.ros.org/en/rolling/About-ROS.md
# Local ROS 2 docs were consulted per project AGENTS.md; this utility plots LeRobot parquet trajectories only.

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_json(path: Path) -> dict[str, Any]:
    if path.exists():
        with path.open("r", encoding="utf-8") as f:
            return json.load(f)
    return {}


def names_from_feature(info: dict[str, Any], key: str, width: int) -> list[str]:
    spec = info.get("features", {}).get(key, {})
    raw_names = spec.get("names") if isinstance(spec, dict) else None
    if isinstance(raw_names, list) and len(raw_names) == width:
        return [str(name).strip() or f"{key}.{i}" for i, name in enumerate(raw_names)]
    if isinstance(raw_names, list) and len(raw_names) == 1 and isinstance(raw_names[0], str):
        parts = [part.strip() for part in raw_names[0].split(",")]
        if len(parts) == width:
            return parts
    return [f"{key}.{i}" for i in range(width)]


def flatten_value(value: Any) -> list[float]:
    if isinstance(value, np.ndarray):
        return value.astype(float).reshape(-1).tolist()
    if isinstance(value, (list, tuple)):
        return np.asarray(value, dtype=float).reshape(-1).tolist()
    return [float(value)]


def expand_columns(df: pd.DataFrame, base_key: str, info: dict[str, Any]) -> pd.DataFrame:
    if base_key in df.columns:
        values = df[base_key].map(flatten_value).tolist()
        width = max((len(row) for row in values), default=0)
        padded = [row + [np.nan] * (width - len(row)) for row in values]
        names = names_from_feature(info, base_key, width)
        return pd.DataFrame(padded, columns=names, index=df.index)

    prefix = f"{base_key}."
    cols = [col for col in df.columns if col.startswith(prefix)]
    if cols:
        out = df[cols].copy()
        out.columns = [col.removeprefix(prefix) for col in cols]
        return out.apply(pd.to_numeric, errors="coerce")

    return pd.DataFrame(index=df.index)


def episode_path(root: Path, episode_index: int) -> Path:
    direct = root / "data" / f"chunk-{episode_index // 1000:03d}" / f"episode_{episode_index:06d}.parquet"
    if direct.exists():
        return direct
    files = sorted((root / "data" / "chunk-000").glob("file-*.parquet"))
    if files:
        return files[0]
    raise FileNotFoundError(f"No parquet file found for episode {episode_index} under {root}")


def maybe_filter_episode(df: pd.DataFrame, episode_index: int) -> pd.DataFrame:
    if "episode_index" in df.columns:
        filtered = df[df["episode_index"] == episode_index]
        if not filtered.empty:
            return filtered.reset_index(drop=True)
    return df.reset_index(drop=True)


def x_axis(df: pd.DataFrame) -> tuple[pd.Series, str]:
    if "timestamp" in df.columns:
        return pd.to_numeric(df["timestamp"], errors="coerce"), "timestamp"
    if "frame_index" in df.columns:
        return pd.to_numeric(df["frame_index"], errors="coerce"), "frame_index"
    return pd.Series(np.arange(len(df))), "row_index"


def plot_timeseries(x: pd.Series, values: pd.DataFrame, xlabel: str, title: str, out_path: Path) -> None:
    fig, ax = plt.subplots(figsize=(14, 7))
    for col in values.columns:
        ax.plot(x, values[col], linewidth=0.9, label=str(col))
    ax.set_title(title)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("value")
    ax.grid(True, alpha=0.25)
    if len(values.columns) <= 20:
        ax.legend(loc="best", fontsize="small", ncols=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)


def find_pose_key(df: pd.DataFrame) -> str | None:
    for key in ("observation.ee_pose", "ee_pose", "observation.state.pose", "action.pose"):
        if key in df.columns or any(col.startswith(f"{key}.") for col in df.columns):
            return key
    return None


def plot_ee_xyz(x: pd.Series, pose_values: pd.DataFrame, xlabel: str, out_path: Path) -> bool:
    lower_cols = [str(col).lower() for col in pose_values.columns]
    xyz_indices = []
    for target in ("x", "y", "z"):
        matches = [i for i, col in enumerate(lower_cols) if col.endswith(f"_{target}") or col == target]
        if matches:
            xyz_indices.append(matches[0])
    if len(xyz_indices) < 3 and pose_values.shape[1] >= 3:
        xyz_indices = [0, 1, 2]
    if len(xyz_indices) < 3:
        return False

    xyz = pose_values.iloc[:, xyz_indices[:3]].copy()
    xyz.columns = ["x", "y", "z"]

    fig = plt.figure(figsize=(12, 8))
    try:
        ax = fig.add_subplot(111, projection="3d")
        ax.plot(xyz["x"], xyz["y"], xyz["z"], linewidth=1.4)
        ax.scatter(xyz["x"].iloc[0], xyz["y"].iloc[0], xyz["z"].iloc[0], label="start", s=35)
        ax.scatter(xyz["x"].iloc[-1], xyz["y"].iloc[-1], xyz["z"].iloc[-1], label="end", s=35)
        ax.set_title("End-effector XYZ trajectory")
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.legend()
    except ValueError:
        plt.close(fig)
        fig, axes = plt.subplots(1, 3, figsize=(15, 5))
        pairs = [("x", "y"), ("x", "z"), ("y", "z")]
        for ax, (a, b) in zip(axes, pairs):
            ax.plot(xyz[a], xyz[b], linewidth=1.2)
            ax.scatter(xyz[a].iloc[0], xyz[b].iloc[0], label="start", s=25)
            ax.scatter(xyz[a].iloc[-1], xyz[b].iloc[-1], label="end", s=25)
            ax.set_xlabel(a)
            ax.set_ylabel(b)
            ax.grid(True, alpha=0.25)
        axes[0].legend()
        fig.suptitle("End-effector XYZ trajectory projections")
    fig.tight_layout()
    fig.savefig(out_path, dpi=160)
    plt.close(fig)

    fig2, ax2 = plt.subplots(figsize=(12, 6))
    for col in xyz.columns:
        ax2.plot(x, xyz[col], label=col, linewidth=1.0)
    ax2.set_title("End-effector XYZ over time")
    ax2.set_xlabel(xlabel)
    ax2.set_ylabel("position")
    ax2.grid(True, alpha=0.25)
    ax2.legend()
    fig2.tight_layout()
    fig2.savefig(out_path.with_name(out_path.stem + "_timeseries.png"), dpi=160)
    plt.close(fig2)
    return True


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot action/state trajectories from a LeRobot parquet episode.")
    parser.add_argument("--dataset-root", type=Path, required=True)
    parser.add_argument("--episode-index", type=int, default=0)
    parser.add_argument("--out-dir", type=Path, default=Path("outputs/lerobot_traj"))
    args = parser.parse_args()

    root = args.dataset_root.resolve()
    out_dir = args.out_dir.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)
    info = load_json(root / "meta" / "info.json")

    df = maybe_filter_episode(pd.read_parquet(episode_path(root, args.episode_index)), args.episode_index)
    x, xlabel = x_axis(df)
    ep = f"ep{args.episode_index:03d}"

    action = expand_columns(df, "action", info)
    state = expand_columns(df, "observation.state", info)
    if action.empty:
        print("warning: no action columns detected")
    else:
        action.to_csv(out_dir / f"{ep}_action.csv", index=False)
        plot_timeseries(x, action, xlabel, f"Action trajectory {ep}", out_dir / f"action_trajectory_{ep}.png")
    if state.empty:
        print("warning: no observation.state columns detected")
    else:
        state.to_csv(out_dir / f"{ep}_state.csv", index=False)
        plot_timeseries(x, state, xlabel, f"State trajectory {ep}", out_dir / f"state_trajectory_{ep}.png")

    pose_key = find_pose_key(df)
    ee_written = False
    if pose_key:
        pose = expand_columns(df, pose_key, info)
        ee_written = plot_ee_xyz(x, pose, xlabel, out_dir / f"ee_xyz_trajectory_{ep}.png")

    print(f"dataset_root: {root}")
    print(f"episode_rows: {len(df)}")
    print(f"x_axis: {xlabel}")
    print(f"action_dim: {action.shape[1]}")
    print(f"state_dim: {state.shape[1]}")
    print(f"pose_key: {pose_key}")
    print(f"ee_xyz_written: {ee_written}")
    print(f"out_dir: {out_dir}")


if __name__ == "__main__":
    main()
