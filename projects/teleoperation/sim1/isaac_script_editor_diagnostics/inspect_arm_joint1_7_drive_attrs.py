"""
Isaac Sim Script Editor diagnostic.

Local documentation referenced:
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/articulation_controller.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/joint_inspector.md
  - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup_tutorials/joint_tuning.md

Purpose:
  Read-only scan of current USD stage joint and drive attributes related to
  left/right arm joint1-7 torque disappearance. Paste into Isaac Sim Script Editor.
  Results are written to one Markdown file in this same diagnostics folder;
  nothing is printed to the Script Editor console.
"""

from datetime import datetime
import os
from pathlib import Path

from pxr import Usd
import omni.usd


OUTPUT_DIR = Path(
    os.environ.get("SIM1_DIR", str(Path(__file__).resolve().parents[1]))
) / "isaac_script_editor_diagnostics"
ROBOT_ROOT = "/World/Robot"
NAME_PATTERNS = (
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
)

ATTR_KEYWORDS = (
    "drive",
    "stiffness",
    "damping",
    "maxforce",
    "maxtorque",
    "force",
    "torque",
    "target",
    "limit",
    "break",
    "enabled",
    "axis",
    "exclude",
)


def _safe_value(attr):
    try:
        value = attr.Get()
    except Exception as exc:
        return f"<GET_FAILED {type(exc).__name__}: {exc}>"
    return repr(value)


def _matches(prim):
    text = prim.GetPath().pathString.lower()
    return any(pattern.lower() in text for pattern in NAME_PATTERNS)


def _interesting_attrs(prim):
    rows = []
    for attr in prim.GetAttributes():
        name = attr.GetName()
        lowered = name.lower()
        if any(keyword in lowered for keyword in ATTR_KEYWORDS):
            rows.append((name, _safe_value(attr)))
    return rows


def _is_joint_like(prim):
    type_name = prim.GetTypeName()
    if "Joint" in type_name:
        return True
    return any("Joint" in schema for schema in prim.GetAppliedSchemas())


def _report_paths(stem):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    return {
        "md_latest": OUTPUT_DIR / f"{stem}_latest.md",
    }


def _write_report(paths, report):
    md_lines = [
        "# arm joint1-7 drive attribute scan",
        "",
        f"- generated_at: `{report['generated_at']}`",
        f"- robot_root: `{report['robot_root']}`",
        f"- status: `{report['status']}`",
        f"- joint_like_count: `{report['joint_like_count']}`",
        f"- matched_prim_count: `{report['matched_prim_count']}`",
        "",
    ]
    if report.get("error"):
        md_lines.extend(["## Error", "", report["error"], ""])

    for prim in report["matched_prims"]:
        md_lines.extend(
            [
                "## Prim",
                "",
                f"- path: `{prim['path']}`",
                f"- type: `{prim['type']}`",
                f"- active: `{prim['active']}`",
                f"- valid: `{prim['valid']}`",
                f"- schemas: `{', '.join(prim['schemas']) if prim['schemas'] else '<none>'}`",
                "",
                "| Attribute | Value |",
                "| --- | --- |",
            ]
        )
        if prim["attributes"]:
            for attr in prim["attributes"]:
                value = str(attr["value"]).replace("\n", "\\n")
                md_lines.append(f"| `{attr['name']}` | `{value}` |")
        else:
            md_lines.append("| `<none>` | `<none>` |")
        md_lines.append("")

    md_lines.extend(
        [
            "## What To Compare",
            "",
            "1. joint1-7 maxForce/maxTorque should not be 0 or unexpectedly tiny.",
            "2. Drive stiffness/damping should match the working joints or intended tuning.",
            "3. Break force/torque should not be tripped/too low.",
            "4. Axis, limits, and enabled flags should match the robot model expectation.",
            "5. If attributes look normal, check ROS command stream and controller mode next.",
            "",
        ]
    )
    paths["md_latest"].write_text("\n".join(md_lines), encoding="utf-8")


def main():
    paths = _report_paths("arm_joint1_7_drive_attrs")
    report = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "robot_root": ROBOT_ROOT,
        "status": "ok",
        "error": "",
        "joint_like_count": 0,
        "matched_prim_count": 0,
        "matched_prims": [],
    }

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        report["status"] = "error"
        report["error"] = "No USD stage is open."
        _write_report(paths, report)
        return

    root = stage.GetPrimAtPath(ROBOT_ROOT)
    if not root.IsValid():
        report["status"] = "error"
        report["error"] = f"Robot root not found: {ROBOT_ROOT}. Edit ROBOT_ROOT and run again."
        _write_report(paths, report)
        return

    matched = []
    joint_like_count = 0
    for prim in Usd.PrimRange(root):
        if _is_joint_like(prim):
            joint_like_count += 1
        if _matches(prim):
            matched.append(prim)

    report["joint_like_count"] = joint_like_count
    report["matched_prim_count"] = len(matched)

    for prim in matched:
        schemas = prim.GetAppliedSchemas()
        prim_record = {
            "path": prim.GetPath().pathString,
            "type": prim.GetTypeName(),
            "active": prim.IsActive(),
            "valid": prim.IsValid(),
            "schemas": list(schemas),
            "attributes": [],
        }

        for name, value in sorted(_interesting_attrs(prim)):
            attr_record = {"name": name, "value": value}
            prim_record["attributes"].append(attr_record)
        report["matched_prims"].append(prim_record)

    _write_report(paths, report)


main()
