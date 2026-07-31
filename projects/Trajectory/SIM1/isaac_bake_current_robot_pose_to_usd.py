#!/usr/bin/env python3
"""Bake the currently visible SIM1 robot link transforms into the open USD stage.

Run this inside Isaac Sim's Window > Script Editor after the robot is visibly in the
desired pose:
exec(open("/home/gtk/isaac_ocs_project/projects/Trajectory/SIM1/isaac_bake_current_robot_pose_to_usd.py").read())

References:
- /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/environment_setup.md
- /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/omniverse_usd/open_usd.md
- /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1/isaac_save_arm_initial_pose_to_usd.py
"""

from __future__ import annotations

import time

import omni.usd
from pxr import Sdf, Usd, UsdGeom


ROBOT_ROOT = "/World/Robot"
BAKE_ATTR = "sim1:bakedInitialVisiblePose"
SAVE_AS_PATH = ""


def is_robot_link_prim(prim) -> bool:
    if not prim or not prim.IsValid():
        return False
    path = str(prim.GetPath())
    if not path.startswith(ROBOT_ROOT + "/"):
        return False
    if path.startswith(ROBOT_ROOT + "/joints"):
        return False
    if prim.GetTypeName() != "Xform":
        return False
    if not prim.GetAttribute("physics:rigidBodyEnabled"):
        return False
    return True


def set_custom_attr(prim, name, type_name, value) -> None:
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)


def bake_local_transform(prim) -> None:
    local_matrix = omni.usd.get_local_transform_matrix(prim)
    xform = UsdGeom.Xformable(prim)
    xform.ClearXformOpOrder()
    op = xform.AddXformOp(UsdGeom.XformOp.TypeTransform, UsdGeom.XformOp.PrecisionDouble, "")
    op.Set(local_matrix)
    set_custom_attr(prim, BAKE_ATTR, Sdf.ValueTypeNames.Bool, True)


def main() -> None:
    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open in Isaac Sim.")

    root_layer = stage.GetRootLayer()
    stage.SetEditTarget(Usd.EditTarget(root_layer))

    robot = stage.GetPrimAtPath(ROBOT_ROOT)
    if not robot:
        raise RuntimeError(f"Cannot find robot root: {ROBOT_ROOT}")

    baked_paths = []
    for prim in Usd.PrimRange(robot):
        if is_robot_link_prim(prim):
            bake_local_transform(prim)
            baked_paths.append(str(prim.GetPath()))

    set_custom_attr(robot, "sim1:bakedInitialVisiblePoseTime", Sdf.ValueTypeNames.Double, float(time.time()))
    set_custom_attr(robot, "sim1:bakedInitialVisiblePoseCount", Sdf.ValueTypeNames.Int, len(baked_paths))

    if SAVE_AS_PATH:
        usd_context.save_as_stage(SAVE_AS_PATH, None)
        print(f"[SIM1] Saved current stage as: {SAVE_AS_PATH}")
    else:
        root_layer.Save()
        print(f"[SIM1] Saved current stage in place: {root_layer.identifier}")

    print(f"[SIM1] Baked {len(baked_paths)} robot link transforms for static initial display.")
    for path in baked_paths:
        print(f"  {path}")


main()
