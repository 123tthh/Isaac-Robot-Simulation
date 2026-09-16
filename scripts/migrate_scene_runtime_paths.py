#!/usr/bin/env python3
# References:
# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omniverse_usd/open_usd.md
# https://docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/environment_setup.md
"""Audit or migrate legacy absolute paths embedded in the canonical USD."""

from __future__ import annotations

import argparse
import shutil
from datetime import datetime
from pathlib import Path

from isaacsim.simulation_app import SimulationApp


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SCENE = PROJECT_ROOT / "assets/scenes/scene.usd"
BACKUP_DIR = PROJECT_ROOT / "backups/scene_runtime_paths_20260724"

REPLACEMENTS = (
    (
        '"LOG_DIR": "/home/gtk/ros2_log"',
        '"LOG_DIR": os.environ.get('
        f'"ROS2_LOG_DIR", "{PROJECT_ROOT / "data/ros2_log"}")',
    ),
    (
        "/home/gtk/teleoperation/",
        f"{PROJECT_ROOT / 'projects/teleoperation'}/",
    ),
    (
        "/home/gtk/ros2_ws/",
        f"{PROJECT_ROOT / 'projects/ros2_ws'}/",
    ),
)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, default=DEFAULT_SCENE)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()

    scene = args.scene.resolve()
    simulation_app = SimulationApp({"headless": True})
    from pxr import Usd

    stage = Usd.Stage.Open(str(scene))
    if stage is None:
        simulation_app.close()
        raise RuntimeError(f"could not open USD: {scene}")

    changes: list[tuple[object, str, str, str]] = []
    for prim in stage.Traverse():
        for attr in prim.GetAttributes():
            value = attr.Get()
            if not isinstance(value, str):
                continue
            migrated = value
            for old, new in REPLACEMENTS:
                migrated = migrated.replace(old, new)
            if migrated != value:
                changes.append((attr, str(prim.GetPath()), attr.GetName(), migrated))

    for _, prim_path, attr_name, migrated in changes:
        print(
            f"{prim_path}.{attr_name}: migrated ({len(migrated)} chars)",
            flush=True,
        )

    if args.apply and changes:
        BACKUP_DIR.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        backup = BACKUP_DIR / f"{scene.name}.{stamp}.bak"
        shutil.copy2(scene, backup)
        print(f"backup={backup}", flush=True)
        for attr, _, _, migrated in changes:
            if not attr.Set(migrated):
                raise RuntimeError(f"failed to set {attr.GetPath()}")
        stage.GetRootLayer().Save()

    print(f"scene={scene}", flush=True)
    print(f"changes={len(changes)}", flush=True)
    print(f"applied={args.apply and bool(changes)}", flush=True)
    simulation_app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
