#!/usr/bin/env python3
"""Generate a standalone R1 URDF and verify that its mesh assets are real."""

from pathlib import Path
import argparse
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT / "projects/ros2_ws/src/robot/urdf/r1_fixed.urdf"
OUTPUT = SOURCE.with_name("r1_fixed_portable.urdf")
SCENES = ROOT / "assets/scenes"
PACKAGE_PREFIX = "package://r1_description/meshes/"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="validate assets without writing")
    args = parser.parse_args()

    source = SOURCE.read_text(encoding="utf-8")
    tree = ET.fromstring(source)
    references = [mesh.attrib["filename"] for mesh in tree.findall(".//mesh")]
    unexpected = [name for name in references if not name.startswith(PACKAGE_PREFIX)]
    if unexpected:
        raise SystemExit(f"Unexpected mesh URI: {unexpected[0]}")

    portable = source.replace(PACKAGE_PREFIX, "../meshes/")
    if not args.check:
        OUTPUT.write_text(portable, encoding="utf-8")

    missing = []
    pointers = []
    for name in sorted(set(references)):
        mesh_path = SOURCE.parent.parent / "meshes" / name.removeprefix(PACKAGE_PREFIX)
        if not mesh_path.is_file():
            missing.append(str(mesh_path.relative_to(ROOT)))
        elif mesh_path.open("rb").read(48).startswith(b"version https://git-lfs.github.com/spec/v1"):
            pointers.append(str(mesh_path.relative_to(ROOT)))

    scene_pointers = []
    for scene_path in SCENES.rglob("*.usd"):
        if scene_path.open("rb").read(48).startswith(b"version https://git-lfs.github.com/spec/v1"):
            scene_pointers.append(str(scene_path.relative_to(ROOT)))

    print(f"URDF: {OUTPUT.relative_to(ROOT)} ({len(references)} refs, {len(set(references))} meshes)")
    print(f"Missing meshes: {len(missing)}; mesh LFS pointers: {len(pointers)}; scene LFS pointers: {len(scene_pointers)}")
    for path in missing + pointers + scene_pointers:
        print(f"  {path}")
    return 1 if missing or pointers or scene_pointers else 0


if __name__ == "__main__":
    raise SystemExit(main())
