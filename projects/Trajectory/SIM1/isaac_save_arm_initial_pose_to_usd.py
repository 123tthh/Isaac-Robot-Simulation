#!/usr/bin/env python3
"""Store and apply the latest SIM1 arm initial pose in the current Isaac Sim USD stage.

Run this file inside Isaac Sim's Window > Script Editor, for example:
exec(open("/workspace/projects/Trajectory/SIM1/isaac_save_arm_initial_pose_to_usd.py").read())

References:
- /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/environment_setup.md
- /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/omniverse_usd/open_usd.md
- /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/python_scripting/robots_simulation.md
- /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/asset_validation.md
- /home/gtk/isaac_ocs_project/projects/Trajectory/SIM1/capture_arm_initial_pose.py
"""

from __future__ import annotations

import json
import math
import os
import time
from pathlib import Path

import omni.usd
from pxr import PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics


SIM1_DIR = Path(os.environ.get("SIM1_DIR", "/workspace/projects/Trajectory/SIM1"))
SNAPSHOT_JSON = SIM1_DIR / "trace_data/initial_poses/arm_initial_pose_latest.json"
POSE_PRIM_PATH = "/World/SIM1_ArmInitialPose"
ROBOT_JOINTS_SCOPE = "/World/Robot/joints"
APPLY_TO_ARM_JOINT_DRIVES = True
APPLY_TO_ARM_JOINT_STATES = True

# The captured pose is mechanically good for holding the pallet, but both wrists need
# a 180 degree roll correction for the persisted USD initial pose.
JOINT_POSITION_OFFSETS_RAD = {
    "left_joint7": math.pi,
    "right_joint7": math.pi,
}

# Empty string means save the currently opened stage in place.
# Set this to a full path if you prefer Save As, for example:
# SAVE_AS_PATH = "/workspace/projects/Trajectory/SIM1/usd/SIM1_with_arm_initial_pose.usd"
SAVE_AS_PATH = ""


def set_attr(prim, name, type_name, value):
    attr = prim.GetAttribute(name)
    if not attr:
        attr = prim.CreateAttribute(name, type_name, custom=True)
    attr.Set(value)


def apply_joint_offsets(joint_names, raw_positions_rad):
    return [
        float(position_rad) + JOINT_POSITION_OFFSETS_RAD.get(joint_name, 0.0)
        for joint_name, position_rad in zip(joint_names, raw_positions_rad)
    ]


def format_home_yaml(joint_names, positions_rad, name="home_1"):
    values = [f"{float(position):.9g}" for position in positions_rad]
    return f"{name}: [{', '.join(values[:7])},\n         {', '.join(values[7:])}]"


def set_drive_target_position(drive, value_deg):
    attr = drive.GetTargetPositionAttr()
    if not attr:
        attr = drive.CreateTargetPositionAttr()
    attr.Set(float(value_deg))


def set_drive_target_velocity(drive, value):
    attr = drive.GetTargetVelocityAttr()
    if not attr:
        attr = drive.CreateTargetVelocityAttr()
    attr.Set(float(value))


def set_joint_state_position(joint_prim, value_deg):
    # USD Physics angular joint-state positions are authored in degrees, matching
    # revolute joint limits and drive target positions.
    joint_state_api = PhysxSchema.JointStateAPI.Apply(joint_prim, "angular")
    position_attr = joint_state_api.GetPositionAttr()
    if not position_attr:
        position_attr = joint_state_api.CreatePositionAttr()
    position_attr.Set(float(value_deg))

    velocity_attr = joint_state_api.GetVelocityAttr()
    if not velocity_attr:
        velocity_attr = joint_state_api.CreateVelocityAttr()
    velocity_attr.Set(0.0)


def find_joint_prim(stage, joint_name):
    direct_path = f"{ROBOT_JOINTS_SCOPE}/{joint_name}"
    prim = stage.GetPrimAtPath(direct_path)
    if prim:
        return prim

    matches = [
        prim
        for prim in stage.TraverseAll()
        if prim.GetName() == joint_name and prim.GetTypeName() == "PhysicsRevoluteJoint"
    ]
    if len(matches) == 1:
        return matches[0]
    if len(matches) > 1:
        raise RuntimeError(
            f"Found multiple revolute joints named {joint_name}: "
            + ", ".join(str(prim.GetPath()) for prim in matches)
        )
    raise RuntimeError(f"Cannot find revolute joint prim for {joint_name}")


def apply_arm_pose_to_usd_joints(stage, joint_names, positions_rad):
    applied = []
    for joint_name, position_rad in zip(joint_names, positions_rad):
        joint_prim = find_joint_prim(stage, joint_name)
        if joint_prim.GetTypeName() != "PhysicsRevoluteJoint":
            raise RuntimeError(f"{joint_prim.GetPath()} is not a PhysicsRevoluteJoint")

        position_deg = math.degrees(float(position_rad))
        if APPLY_TO_ARM_JOINT_STATES:
            set_joint_state_position(joint_prim, position_deg)

        if APPLY_TO_ARM_JOINT_DRIVES:
            drive = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
            if not drive:
                drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            set_drive_target_position(drive, position_deg)
            set_drive_target_velocity(drive, 0.0)

        set_attr(joint_prim, "sim1:initialPositionRad", Sdf.ValueTypeNames.Double, float(position_rad))
        set_attr(joint_prim, "sim1:initialPositionDeg", Sdf.ValueTypeNames.Double, float(position_deg))
        applied.append((str(joint_prim.GetPath()), position_rad, position_deg))
    return applied


def main() -> None:
    if not SNAPSHOT_JSON.exists():
        raise FileNotFoundError(f"Missing snapshot JSON: {SNAPSHOT_JSON}")

    with SNAPSHOT_JSON.open("r", encoding="utf-8") as f:
        snapshot = json.load(f)

    joint_names = list(snapshot["joint_order"])
    positions_map = snapshot["positions_rad"]
    raw_positions_rad = [float(positions_map[name]) for name in joint_names]
    positions_rad = apply_joint_offsets(joint_names, raw_positions_rad)
    ocs2_home_yaml_raw = str(snapshot.get("ocs2_home_yaml", ""))
    ocs2_home_yaml_applied = format_home_yaml(joint_names, positions_rad)

    usd_context = omni.usd.get_context()
    stage = usd_context.get_stage()
    if stage is None:
        raise RuntimeError("No USD stage is open in Isaac Sim.")
    root_layer = stage.GetRootLayer()
    stage.SetEditTarget(Usd.EditTarget(root_layer))

    prim = stage.GetPrimAtPath(POSE_PRIM_PATH)
    if not prim:
        prim = UsdGeom.Xform.Define(stage, POSE_PRIM_PATH).GetPrim()

    set_attr(prim, "sim1:poseName", Sdf.ValueTypeNames.String, "arm_initial_pose")
    set_attr(prim, "sim1:createdWallTime", Sdf.ValueTypeNames.Double, float(time.time()))
    set_attr(prim, "sim1:sourceSnapshotJson", Sdf.ValueTypeNames.String, str(SNAPSHOT_JSON))
    set_attr(prim, "sim1:jointNames", Sdf.ValueTypeNames.StringArray, joint_names)
    set_attr(prim, "sim1:jointPositionsRadRaw", Sdf.ValueTypeNames.DoubleArray, raw_positions_rad)
    set_attr(prim, "sim1:jointPositionsRad", Sdf.ValueTypeNames.DoubleArray, positions_rad)
    set_attr(prim, "sim1:joint7OffsetRad", Sdf.ValueTypeNames.Double, float(math.pi))
    set_attr(prim, "sim1:ocs2HomeYamlRaw", Sdf.ValueTypeNames.String, ocs2_home_yaml_raw)
    set_attr(prim, "sim1:ocs2HomeYaml", Sdf.ValueTypeNames.String, ocs2_home_yaml_applied)

    applied = []
    if APPLY_TO_ARM_JOINT_DRIVES:
        applied = apply_arm_pose_to_usd_joints(stage, joint_names, positions_rad)

    if SAVE_AS_PATH:
        usd_context.save_as_stage(SAVE_AS_PATH, None)
        print(f"[SIM1] Saved current stage as: {SAVE_AS_PATH}")
    else:
        root_layer.Save()
        print(f"[SIM1] Saved current stage in place: {root_layer.identifier}")

    print(f"[SIM1] Stored arm initial pose at USD prim: {POSE_PRIM_PATH}")
    if applied:
        print("[SIM1] Applied arm pose to angular joint state and drive targetPosition:")
        for joint_path, position_rad, position_deg in applied:
            print(f"  {joint_path}: {position_rad:.9g} rad = {position_deg:.9g} deg")
    print(ocs2_home_yaml_applied)


main()
