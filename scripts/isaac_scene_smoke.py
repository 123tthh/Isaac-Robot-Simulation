#!/usr/bin/env python3
"""Open the project scene in Isaac Sim, step physics, and inspect R1 articulation."""

from pathlib import Path
import json
import os
import traceback

from isaacsim.simulation_app import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
SCENE = Path(os.environ.get("ISAAC_USD_PATH", str(ROOT / "assets/scenes/scene.usd"))).resolve()
REPORT = ROOT / "outputs/isaac_scene_smoke.json"


def main() -> int:
    app = SimulationApp({"headless": True})
    try:
        print("[smoke] application ready", flush=True)
        import omni.timeline
        import omni.usd
        import omni.kit.app
        from pxr import UsdPhysics
        from omni.isaac.dynamic_control import _dynamic_control

        context = omni.usd.get_context()
        if os.environ.get("ISAAC_ENABLE_ROS_BRIDGE") == "1":
            omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate("isaacsim.ros2.bridge", True)
            app.update()
            print("[smoke] bridge enabled", flush=True)
        print(f"[smoke] opening {SCENE}", flush=True)
        if not context.open_stage(str(SCENE)):
            raise RuntimeError(f"Could not open scene: {SCENE}")
        print("[smoke] open_stage returned", flush=True)
        for _ in range(60):
            app.update()
        stage = context.get_stage()
        print("[smoke] stage updates complete", flush=True)
        if stage is None:
            raise RuntimeError("Scene stage did not become available")

        robot = stage.GetPrimAtPath("/World/Robot")
        articulations = [str(p.GetPath()) for p in stage.Traverse() if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
        joints = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
        layers = [x.realPath or x.identifier for x in stage.GetUsedLayers()]

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        print("[smoke] playing", flush=True)
        for _ in range(120):
            app.update()
        timeline.stop()
        print("[smoke] stopped after 120 frames", flush=True)

        dc = _dynamic_control.acquire_dynamic_control_interface()
        handle = dc.get_articulation(articulations[0]) if articulations else 0
        report = {
            "scene": str(SCENE),
            "robot_prim_valid": bool(robot and robot.IsValid()),
            "articulation_roots": articulations,
            "joint_count": len(joints),
            "used_layers": layers,
            "timeline_steps": 120,
            "ros_bridge_requested": os.environ.get("ISAAC_ENABLE_ROS_BRIDGE") == "1",
            "dynamic_control_articulation": int(handle),
        }
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2), flush=True)
        return 0 if report["robot_prim_valid"] and articulations and int(handle) != 0 else 1
    except BaseException:
        # Native shutdown errors must not hide the original Python failure.
        traceback.print_exc()
        raise
    finally:
        print("[smoke] closing application", flush=True)
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
