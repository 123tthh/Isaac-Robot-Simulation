#!/usr/bin/env python3
# Reference: /home/gtk/ai_docs/docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.md

from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def make_mujoco_urdf(urdf_path: Path, robot_pkg_root: Path, strip_meshes: bool) -> Path:
    text = urdf_path.read_text().replace(
        "package://r1_description/meshes/",
        str(robot_pkg_root / "meshes") + "/",
    )
    tmp_dir = Path(tempfile.mkdtemp(prefix="r1_mujoco_urdf_"))
    if strip_meshes:
        root = ET.fromstring(text)
        for link in root.findall("link"):
            for tag in ("visual", "collision"):
                for child in list(link.findall(tag)):
                    geometry = child.find("geometry")
                    if geometry is not None and geometry.find("mesh") is not None:
                        link.remove(child)
        text = ET.tostring(root, encoding="unicode")
    else:
        # MuJoCo's URDF importer may resolve package mesh paths by basename only.
        for mesh in (robot_pkg_root / "meshes").rglob("*"):
            if mesh.is_file():
                link = tmp_dir / mesh.name
                if not link.exists():
                    link.symlink_to(mesh)
    out = tmp_dir / "r1_fixed_mujoco.urdf"
    out.write_text(text)
    return out


def main() -> None:
    ros2_ws = os.environ.get("ROS2_WS", "/workspace/projects/ros2_ws")
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", default=f"{ros2_ws}/src/robot/urdf/r1_fixed.urdf")
    parser.add_argument("--robot-pkg-root", default=f"{ros2_ws}/src/robot")
    parser.add_argument("--keep-meshes", action="store_true")
    parser.add_argument(
        "--npz",
        default=os.environ.get("LEROBOT_NPZ_PATH", ""),
    )
    args = parser.parse_args()
    if not args.npz:
        parser.error("set --npz or LEROBOT_NPZ_PATH to a v3.0-compatible NPZ file")

    urdf_for_mujoco = make_mujoco_urdf(
        Path(args.urdf), Path(args.robot_pkg_root), strip_meshes=not args.keep_meshes
    )
    model = mujoco.MjModel.from_xml_path(str(urdf_for_mujoco))
    data = mujoco.MjData(model)
    print(f"MuJoCo loaded: nq={model.nq} nv={model.nv} nu={model.nu} njnt={model.njnt}")

    trajectory = np.load(args.npz, allow_pickle=True)
    joint_names = [str(name) for name in trajectory["joint_names"].tolist()]
    q = np.asarray(trajectory["q"], dtype=np.float64)
    applied = []
    for name in joint_names:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if joint_id < 0:
            continue
        qpos_addr = model.jnt_qposadr[joint_id]
        data.qpos[qpos_addr] = q[0, joint_names.index(name)]
        applied.append(name)
    mujoco.mj_forward(model, data)
    print(f"Applied first-frame joints: {len(applied)}/{len(joint_names)}")
    if len(applied) != len(joint_names):
        missing = sorted(set(joint_names) - set(applied))
        print("Missing MuJoCo joints:", ", ".join(missing))


if __name__ == "__main__":
    main()
