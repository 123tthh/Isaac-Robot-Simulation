#!/usr/bin/env python3
"""Run the project scene with ROS control graphs and six session cameras.

Reference: Isaac Sim 5.1 standalone_examples/api/isaacsim.ros2.bridge/carter_stereo.py
Bridge extension registration requires an application update before stage loading.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import signal
import time
import traceback

ROOT = Path(__file__).resolve().parents[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("scene", nargs="?", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--duration", type=float, default=0, help="Wall seconds after readiness; 0 runs until closed.")
    parser.add_argument("--no-play", action="store_true")
    args, kit_args = parser.parse_known_args()
    scene = (args.scene or Path(os.environ.get("ISAAC_USD_PATH", ROOT / "assets/scenes/scene_portable.usda"))).expanduser().resolve()
    os.environ["ISAAC_OCS_PROJECT_ROOT"] = str(ROOT)
    os.environ["ISAAC_USD_PATH"] = str(scene)
    os.environ.setdefault("ROS2_LOG_DIR", str(ROOT / "data/ros2_log"))
    os.environ["OCS_CAMERA_REBUILD_AUTOPLAY"] = "0"
    if not scene.is_file():
        raise FileNotFoundError(scene)

    from isaacsim.simulation_app import SimulationApp
    app = SimulationApp({"headless": args.headless, "extra_args": kit_args})
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        import carb
        import omni.graph.core as og
        import omni.kit.app
        import omni.timeline
        import omni.usd

        manager = omni.kit.app.get_app().get_extension_manager()
        for name in ("isaacsim.ros2.bridge", "isaacsim.ros2.sim_control"):
            manager.set_extension_enabled_immediate(name, True)
        # User-authorized project scripts, scoped to this Kit session.
        carb.settings.get_settings().set_bool("/app/omni.graph.scriptnode/opt_in", True)
        app.update()
        print("[SIM1] ROS extensions registered; opening project stage", flush=True)

        spec = importlib.util.spec_from_file_location("sim1_camera_session", ROOT / "scripts/rebuild_camera_render_products.py")
        cameras = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cameras)
        deadline = time.monotonic() + 180
        while not cameras.STARTUP_TASK.done():
            if stopping or time.monotonic() > deadline:
                raise TimeoutError("SIM1 camera initialization interrupted or timed out")
            app.update()
        result = cameras.STARTUP_TASK.result()
        if result["status"] != "ok":
            raise RuntimeError(f"Camera initialization failed: {result['failures']}")

        stage = omni.usd.get_context().get_stage()
        stage.SetEditTarget(stage.GetSessionLayer())
        # Interactive teleoperation must advance with wall time even when six
        # cameras render below 60 FPS. The authored minFrameRate=60 otherwise
        # allows only one 1/60 s physics step per rendered frame.
        settings = carb.settings.get_settings()
        settings.set_bool("/app/player/useFixedTimeStepping", False)
        settings.set_int("/persistent/simulation/minFrameRate", 5)
        settings.set_int("/app/renderer/sleepMsOutOfFocus", 0)
        settings.set_int("/app/renderer/sleepMsOnFocus", 0)
        # The saved telemetry graph publishes joints and clock, but no odometry.
        # Follow Isaac's test_ros2_odometry.py using the actual chassis rigid body.
        import usdrt
        keys = og.Controller.Keys
        og.Controller.edit(
            {"graph_path": "/World/ActionGraphs/SIM1_Odometry", "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: [("tick", "omni.graph.action.OnPlaybackTick"),
                    ("time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ("context", "isaacsim.ros2.bridge.ROS2Context"),
                    ("compute", "isaacsim.core.nodes.IsaacComputeOdometry"),
                    ("publish", "isaacsim.ros2.bridge.ROS2PublishOdometry")],
                keys.SET_VALUES: [("compute.inputs:chassisPrim", [usdrt.Sdf.Path("/World/Robot/base_link")]),
                    ("context.inputs:useDomainIDEnvVar", True),
                    ("publish.inputs:topicName", "odom"),
                    ("publish.inputs:chassisFrameId", "base_link"),
                    ("publish.inputs:odomFrameId", "odom")],
                keys.CONNECT: [("tick.outputs:tick", "compute.inputs:execIn"),
                    ("compute.outputs:execOut", "publish.inputs:execIn"),
                    ("context.outputs:context", "publish.inputs:context"),
                    ("time.outputs:simulationTime", "publish.inputs:timeStamp")]
                    + [(f"compute.outputs:{field}", f"publish.inputs:{field}")
                       for field in ("position", "orientation", "linearVelocity", "angularVelocity")],
            },
        )
        for graph in og.get_all_graphs():
            print(f"[SIM1] graph {graph.get_path_to_graph()} disabled={graph.is_disabled()}", flush=True)
        if not args.no_play:
            omni.timeline.get_timeline_interface().play()
        print("[SIM1] READY: scene, ROS Bridge, control graphs and six camera products", flush=True)
        ready = time.monotonic()
        frames = 0
        next_status = ready
        while app.is_running() and not stopping:
            if args.duration > 0 and time.monotonic() - ready >= args.duration:
                break
            app.update()
            frames += 1
            if time.monotonic() >= next_status:
                timeline = omni.timeline.get_timeline_interface()
                print(json.dumps({"status": "running", "playing": timeline.is_playing(),
                    "simulation_time": timeline.get_current_time(), "frames": frames}), flush=True)
                next_status = time.monotonic() + 10
        print(json.dumps({"status": "stopped", "frames": frames, "wall_seconds": time.monotonic() - ready}), flush=True)
        omni.timeline.get_timeline_interface().stop()
        for _ in range(5):
            app.update()
        return 0
    except BaseException:
        traceback.print_exc()
        raise
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
