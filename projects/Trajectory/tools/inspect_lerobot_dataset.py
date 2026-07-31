#!/usr/bin/env python3
# Reference: /home/gtk/ai_docs/docs.ros.org/en/rolling/About-ROS.md
# Local ROS 2 docs were consulted per project AGENTS.md; this utility inspects LeRobot files only.

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import pandas as pd


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {}
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def read_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    rows = []
    with path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if line:
                rows.append(json.loads(line))
    return rows


def find_dataset_root(start: Path) -> Path:
    preferred = start / "place_tray_middle"
    if (preferred / "meta" / "info.json").exists() and (preferred / "data" / "chunk-000").is_dir():
        return preferred.resolve()

    candidates = []
    for info_path in start.rglob("meta/info.json"):
        root = info_path.parents[1]
        if (root / "data" / "chunk-000").is_dir():
            candidates.append(root)
    if not candidates:
        raise FileNotFoundError("No dataset root containing meta/info.json and data/chunk-000 was found.")
    return sorted(candidates, key=lambda p: (len(p.parts), str(p)))[0].resolve()


def detect_version(root: Path) -> str:
    v21 = bool(list((root / "data" / "chunk-000").glob("episode_*.parquet"))) and (
        root / "meta" / "episodes.jsonl"
    ).exists()
    v30 = bool(list((root / "data" / "chunk-000").glob("file-*.parquet"))) and bool(
        list((root / "meta" / "episodes" / "chunk-000").glob("file-*.parquet"))
    )
    if v30:
        return "v3.0-like"
    if v21:
        return "v2.1-like"
    return "unknown"


def feature_keys_by_kind(features: dict[str, Any]) -> tuple[list[str], list[str], list[str]]:
    camera_keys = []
    action_keys = []
    state_keys = []
    for key, spec in features.items():
        dtype = spec.get("dtype") if isinstance(spec, dict) else None
        if dtype == "video" or key.startswith("observation.image") or key.startswith("observation.images"):
            camera_keys.append(key)
        if key == "action" or key.startswith("action."):
            action_keys.append(key)
        if key == "observation.state" or key.startswith("observation.state."):
            state_keys.append(key)
    return camera_keys, action_keys, state_keys


def first_episode_path(root: Path, version: str) -> Path | None:
    if version == "v3.0-like":
        files = sorted((root / "data" / "chunk-000").glob("file-*.parquet"))
    else:
        files = sorted((root / "data" / "chunk-000").glob("episode_*.parquet"))
    return files[0] if files else None


def main() -> None:
    parser = argparse.ArgumentParser(description="Inspect a local LeRobot dataset directory.")
    parser.add_argument("--dataset-root", type=Path, default=None, help="Dataset root. Auto-detected if omitted.")
    parser.add_argument("--start-dir", type=Path, default=Path.cwd(), help="Directory used for auto-detection.")
    args = parser.parse_args()

    root = args.dataset_root.resolve() if args.dataset_root else find_dataset_root(args.start_dir.resolve())
    info = read_json(root / "meta" / "info.json")
    modality = read_json(root / "meta" / "modality.json")
    episodes = read_jsonl(root / "meta" / "episodes.jsonl")
    episodes_stats = read_jsonl(root / "meta" / "episodes_stats.jsonl")
    version = detect_version(root)

    features = info.get("features", {})
    camera_keys, action_keys, state_keys = feature_keys_by_kind(features)
    total_episodes = info.get("total_episodes") or len(episodes)
    total_frames = info.get("total_frames") or sum(ep.get("length", 0) for ep in episodes)

    print(f"dataset_root: {root}")
    print(f"detected_version: {version}")
    print(f"codebase_version: {info.get('codebase_version')}")
    print(f"dataset_version: {info.get('dataset_version') or info.get('version')}")
    print(f"fps: {info.get('fps')}")
    print(f"total_episodes: {total_episodes}")
    print(f"total_frames: {total_frames}")
    print("features:")
    for key, value in features.items():
        print(f"  - {key}: {value}")
    print(f"camera_keys: {camera_keys}")
    print(f"action_key: {action_keys[0] if action_keys else None}")
    print(f"action_keys: {action_keys}")
    print(f"state_key: {state_keys[0] if state_keys else None}")
    print(f"state_keys: {state_keys}")
    print(f"modality_keys: {list(modality.keys())}")
    print(f"episodes_jsonl_rows: {len(episodes)}")
    print(f"episodes_stats_jsonl_rows: {len(episodes_stats)}")

    parquet_path = first_episode_path(root, version)
    if parquet_path is None:
        print("first_parquet: None")
        return

    df = pd.read_parquet(parquet_path)
    print(f"first_parquet: {parquet_path}")
    print(f"shape: {df.shape}")
    print(f"columns: {list(df.columns)}")
    print("dtypes:")
    print(df.dtypes.to_string())
    print("head:")
    print(df.head().to_string(max_cols=30, max_colwidth=120))


if __name__ == "__main__":
    main()
