#!/usr/bin/env python3
"""Run 615scene briefly and keep its ROS camera graph on the timeline.

Local documentation references:
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/omniverse_usd/open_usd.md
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_getting_started.md
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

from isaacsim.simulation_app import SimulationApp


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("usd_path", type=Path)
    parser.add_argument("--seconds", type=float, default=20.0)
    args = parser.parse_args()

    app = SimulationApp({"headless": True})
    try:
        import omni.timeline
        import omni.usd

        context = omni.usd.get_context()
        context.open_stage(str(args.usd_path.resolve()))
        deadline = time.monotonic() + 60.0
        while context.is_loading() and time.monotonic() < deadline:
            app.update()
        if context.is_loading():
            raise RuntimeError("USD stage did not finish loading")
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        print(f"playing={timeline.is_playing()}", flush=True)
        end = time.monotonic() + args.seconds
        while time.monotonic() < end:
            app.update()
        timeline.stop()
        print("stopped", flush=True)
    finally:
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
