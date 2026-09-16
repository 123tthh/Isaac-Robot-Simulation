#!/usr/bin/env python3
"""Create an OmniSim import variant with dynamic joints attached to fixed-chain leaders."""

from pathlib import Path
import math
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "projects/ros2_ws/src/robot/urdf/r1_fixed_portable.urdf"
OUTPUT = SOURCE.with_name("r1_fixed_omnisim.urdf")


def matmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def origin_matrix(elem):
    if elem is None:
        xyz = [0.0, 0.0, 0.0]
        roll = pitch = yaw = 0.0
    else:
        xyz = [float(x) for x in elem.attrib.get("xyz", "0 0 0").split()]
        roll, pitch, yaw = [float(x) for x in elem.attrib.get("rpy", "0 0 0").split()]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0.0, 0.0, 0.0, 1.0],
    ]


def matrix_origin(m):
    pitch = math.atan2(-m[2][0], math.hypot(m[0][0], m[1][0]))
    if abs(math.cos(pitch)) < 1e-9:
        roll = 0.0
        yaw = math.atan2(-m[0][1], m[1][1])
    else:
        roll = math.atan2(m[2][1], m[2][2])
        yaw = math.atan2(m[1][0], m[0][0])
    xyz = [m[i][3] for i in range(3)]
    return {"xyz": " ".join(f"{x:.12g}" for x in xyz), "rpy": " ".join(f"{x:.12g}" for x in (roll, pitch, yaw))}


def main() -> int:
    tree = ET.parse(SOURCE)
    robot = tree.getroot()
    joints = robot.findall("joint")
    parent_joint = {joint.find("child").attrib["link"]: joint for joint in joints}
    changed = []
    for joint in joints:
        if joint.attrib["type"] == "fixed":
            continue
        parent_elem = joint.find("parent")
        original_parent = parent_elem.attrib["link"]
        parent = original_parent
        fixed_chain = []
        while parent in parent_joint and parent_joint[parent].attrib["type"] == "fixed":
            fixed_joint = parent_joint[parent]
            fixed_chain.append(fixed_joint)
            parent = fixed_joint.find("parent").attrib["link"]
        if not fixed_chain:
            continue
        transform = origin_matrix(None)
        for fixed_joint in reversed(fixed_chain):
            transform = matmul(transform, origin_matrix(fixed_joint.find("origin")))
        transform = matmul(transform, origin_matrix(joint.find("origin")))
        parent_elem.attrib["link"] = parent
        origin = joint.find("origin")
        if origin is None:
            origin = ET.Element("origin")
            joint.insert(0, origin)
        origin.attrib.update(matrix_origin(transform))
        changed.append((joint.attrib["name"], original_parent, parent))

    ET.indent(tree, space="  ")
    tree.write(OUTPUT, encoding="utf-8", xml_declaration=True)
    for name, before, after in changed:
        print(f"{name}: {before} -> {after}")
    print(f"Created {OUTPUT.relative_to(ROOT)}; rewired {len(changed)} dynamic joints")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
