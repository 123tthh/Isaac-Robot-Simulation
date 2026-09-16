#!/usr/bin/env python3
"""Import the standalone R1 URDF in Isaac Sim without sourcing ROS."""

from pathlib import Path
import json

from isaacsim.simulation_app import SimulationApp


ROOT = Path(__file__).resolve().parents[1]
URDF = ROOT / "projects/ros2_ws/src/robot/urdf/r1_fixed_portable.urdf"
REPORT = ROOT / "outputs/isaac_urdf_import.json"


def main() -> int:
    app = SimulationApp({"headless": True})
    try:
        import omni.kit.app
        import omni.kit.commands
        import omni.usd
        from pxr import UsdGeom, UsdPhysics

        omni.kit.app.get_app().get_extension_manager().set_extension_enabled_immediate(
            "isaacsim.asset.importer.urdf", True
        )
        context = omni.usd.get_context()
        context.new_stage()
        status, config = omni.kit.commands.execute("URDFCreateImportConfig")
        if not status:
            raise RuntimeError("URDF import configuration failed")
        config.merge_fixed_joints = True
        config.fix_base = False
        config.import_inertia_tensor = True
        status, path = omni.kit.commands.execute(
            "URDFParseAndImportFile", urdf_path=str(URDF), import_config=config
        )
        if not status:
            raise RuntimeError(f"URDF import failed: {path}")
        for _ in range(30):
            app.update()
        stage = context.get_stage()
        robot = stage.GetPrimAtPath(path)
        if not robot or not robot.IsValid():
            raise RuntimeError(f"Imported root not present: {path}")
        meshes = [UsdGeom.Mesh(p) for p in stage.Traverse() if p.IsA(UsdGeom.Mesh)]
        empty = [str(x.GetPath()) for x in meshes if not x.GetPointsAttr().Get()]
        joints = [str(p.GetPath()) for p in stage.Traverse() if p.IsA(UsdPhysics.Joint)]
        arms = {name: bool(stage.GetPrimAtPath(f"{path}/{name}")) for name in
                ("left_Link1", "right_Link1", "left_PGIA_link1", "right_PGIA_link1")}
        report = {
            "urdf": str(URDF.relative_to(ROOT)),
            "imported_root": path,
            "mesh_prims": len(meshes),
            "empty_mesh_prims": empty,
            "usd_joint_prims": len(joints),
            "arm_link_prims": arms,
        }
        REPORT.parent.mkdir(parents=True, exist_ok=True)
        REPORT.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, indent=2))
        return 0 if meshes and not empty and joints and all(arms.values()) else 1
    finally:
        app.close()


if __name__ == "__main__":
    raise SystemExit(main())
