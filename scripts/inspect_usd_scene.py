#!/usr/bin/env python3
"""Inspect the structure of an Isaac Sim USD scene without modifying it.

Local documentation references:
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.simulation_app/docs/api.md
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omniverse_usd/open_usd.md
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path

from isaacsim.simulation_app import SimulationApp


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read an Isaac Sim USD and summarize control/camera-related prims."
    )
    parser.add_argument("usd_path", type=Path)
    parser.add_argument(
        "--show-camera-attributes",
        action="store_true",
        default=os.environ.get("ISAAC_OCS_SHOW_CAMERA_ATTRIBUTES") == "1",
        help="Print serialized attributes for Camera_Publish_Graph nodes.",
    )
    parser.add_argument("--show-graph-connections", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    usd_path = args.usd_path.expanduser().resolve()
    if not usd_path.is_file():
        raise FileNotFoundError(usd_path)

    simulation_app = SimulationApp({"headless": True})
    try:
        from pxr import Usd

        stage = Usd.Stage.Open(str(usd_path))
        if stage is None:
            raise RuntimeError(f"Could not open USD: {usd_path}")

        prims = list(stage.Traverse())
        default_prim = stage.GetDefaultPrim()
        keywords = (
            "graph",
            "camera",
            "robot",
            "arm",
            "gripper",
            "r1",
            "mobile",
        )
        matching_prims = [
            (str(prim.GetPath()), prim.GetTypeName())
            for prim in prims
            if any(keyword in str(prim.GetPath()).lower() for keyword in keywords)
        ]

        print(f"usd_path={usd_path}")
        print(f"default_prim={default_prim.GetPath() if default_prim else '<none>'}")
        print(f"prim_count={len(prims)}")
        print(f"matching_prim_count={len(matching_prims)}")
        for prim_path, type_name in matching_prims:
            print(f"{prim_path}\t{type_name}")

        if args.show_camera_attributes:
            graph_prefix = "/World/ActionGraphs/Camera_Publish_Graph/"
            for prim_path, type_name in matching_prims:
                if not prim_path.startswith(graph_prefix) or type_name != "OmniGraphNode":
                    continue
                prim = stage.GetPrimAtPath(prim_path)
                print(f"camera_node={prim_path}")
                for attr in prim.GetAttributes():
                    name = attr.GetName().lower()
                    if any(
                        token in name
                        for token in ("topic", "qos", "render", "type", "camera", "enable", "frame")
                    ):
                        print(f"camera_attr={prim_path}\t{attr.GetName()}\t{attr.Get()}")
        if args.show_graph_connections:
            graph_prefix = "/World/ActionGraphs/Camera_Publish_Graph"
            for prim in stage.Traverse():
                if not str(prim.GetPath()).startswith(graph_prefix):
                    continue
                for attr in prim.GetAttributes():
                    if attr.HasConnectedSource() or attr.GetConnections():
                        print(
                            f"graph_connection={prim.GetPath()}\t{attr.GetName()}\t"
                            f"sources={list(attr.GetConnections())}",
                            flush=True,
                        )
    finally:
        simulation_app.close()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
