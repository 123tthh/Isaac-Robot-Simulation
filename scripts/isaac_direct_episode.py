#!/usr/bin/env python3
"""Record a short direct R1 joint episode while the ROS controller is unavailable."""

from pathlib import Path
import csv
import json

from isaacsim.simulation_app import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
SCENE = ROOT / "assets/scenes/scene_portable.usda"
OUT = ROOT / "outputs/isaac_direct_episode"
ROOT_PRIM = "/World/Robot/base_link"


def main() -> int:
    OUT.mkdir(parents=True, exist_ok=True)
    app = SimulationApp({"headless": True})
    try:
        import omni.timeline
        import omni.usd
        from omni.isaac.dynamic_control import _dynamic_control

        context = omni.usd.get_context()
        if not context.open_stage(str(SCENE)):
            raise RuntimeError(f"Could not open {SCENE}")
        for _ in range(60):
            app.update()
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        for _ in range(20):
            app.update()

        dc = _dynamic_control.acquire_dynamic_control_interface()
        art = dc.get_articulation(ROOT_PRIM)
        if art == _dynamic_control.INVALID_HANDLE:
            raise RuntimeError(f"No articulation at {ROOT_PRIM}")
        dofs = {}
        for index in range(dc.get_articulation_dof_count(art)):
            handle = dc.get_articulation_dof(art, index)
            dofs[dc.get_dof_name(handle)] = handle
        selected = ("left_joint1", "left_PGIA_joint1", "left_PGIA_joint2")
        if any(name not in dofs for name in selected):
            raise RuntimeError(f"Missing R1 DOF; available: {sorted(dofs)}")

        rows = []
        phases = [
            ("baseline", {}, 30),
            ("gripper_open", {"left_PGIA_joint1": 0.08, "left_PGIA_joint2": 0.08}, 90),
            ("arm_move", {"left_joint1": 0.15}, 90),
            ("gripper_close", {"left_PGIA_joint1": 0.04, "left_PGIA_joint2": 0.04}, 90),
        ]
        step = 0
        for phase, targets, frames in phases:
            for name, target in targets.items():
                dc.set_dof_position_target(dofs[name], target)
            for _ in range(frames):
                app.update()
                rows.append({
                    "step": step,
                    "phase": phase,
                    **{f"{name}_position": float(dc.get_dof_position(dofs[name])) for name in selected},
                    **{f"{name}_target": float(dc.get_dof_position_target(dofs[name])) for name in selected},
                })
                step += 1
        timeline.stop()

        csv_path = OUT / "joint_trace.csv"
        with csv_path.open("w", newline="", encoding="utf-8") as file:
            writer = csv.DictWriter(file, fieldnames=rows[0].keys())
            writer.writeheader()
            writer.writerows(rows)
        final = rows[-1]
        report = {
            "scene": str(SCENE),
            "articulation_root": ROOT_PRIM,
            "dof_count": len(dofs),
            "selected_dofs": list(selected),
            "frames": len(rows),
            "final_positions": {name: final[f"{name}_position"] for name in selected},
            "final_targets": {name: final[f"{name}_target"] for name in selected},
            "trace": str(csv_path.relative_to(ROOT)),
        }
        (OUT / "report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
