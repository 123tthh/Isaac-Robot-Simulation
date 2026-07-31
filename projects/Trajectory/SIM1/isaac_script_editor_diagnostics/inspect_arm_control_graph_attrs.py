"""
Isaac Sim Script Editor diagnostic.

Local documentation referenced:
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_simulation/articulation_controller.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.md
  - /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/physics/joint_inspector.md

Purpose:
  Read-only scan of current USD stage for OmniGraph/ROS/action graph attributes
  that mention /arm_joint_cmd, joint names, or command inputs. Paste into
  Isaac Sim Script Editor.
  Results are written to one Markdown file in this same diagnostics folder;
  nothing is printed to the Script Editor console.
"""

from datetime import datetime
import os
from pathlib import Path

from pxr import Usd
import omni.usd


OUTPUT_DIR = Path(
    os.environ.get("SIM1_DIR", "/workspace/projects/Trajectory/SIM1")
) / "isaac_script_editor_diagnostics"
SEARCH_TEXT = (
    "arm_joint_cmd",
    "jointnames",
    "joint_names",
    "positioncommand",
    "velocitycommand",
    "effortcommand",
    "targetprim",
    "topicname",
    "message",
)


def _safe_value(attr):
    try:
        value = attr.Get()
    except Exception as exc:
        return f"<GET_FAILED {type(exc).__name__}: {exc}>"
    return repr(value)


def _safe_connections(attr):
    try:
        return [str(path) for path in attr.GetConnections()]
    except Exception as exc:
        return [f"<GET_CONNECTIONS_FAILED {type(exc).__name__}: {exc}>"]


def _attr_mentions(attr):
    name = attr.GetName().lower()
    if any(token in name for token in SEARCH_TEXT):
        return True
    value = _safe_value(attr).lower()
    if any(token in value for token in SEARCH_TEXT):
        return True
    connections = " ".join(_safe_connections(attr)).lower()
    return any(token in connections for token in SEARCH_TEXT)


def _report_paths(stem):
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    return {
        "md_latest": OUTPUT_DIR / f"{stem}_latest.md",
    }


def _write_report(paths, report):
    md_lines = [
        "# Arm control graph / ROS command attribute scan",
        "",
        f"- generated_at: `{report['generated_at']}`",
        f"- status: `{report['status']}`",
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
                f"- schemas: `{', '.join(prim['schemas']) if prim['schemas'] else '<none>'}`",
                "",
                "| Attribute | Value | Connections |",
                "| --- | --- | --- |",
            ]
        )
        for attr in prim["attributes"]:
            value = str(attr["value"]).replace("\n", "\\n")
            connections = "<none>"
            if attr["connections"]:
                connections = "<br>".join(f"`{conn}`" for conn in attr["connections"])
            md_lines.append(f"| `{attr['name']}` | `{value}` | {connections} |")
        md_lines.append("")

    md_lines.extend(
        [
            "## What To Verify",
            "",
            "1. ROS subscriber topic should be `/arm_joint_cmd`.",
            "2. Articulation action node should command the expected input type.",
            "3. `jointNames` order must match the 14 arm joints expected by OCS2.",
            "4. arm joint1-7 names must be present and not accidentally omitted.",
            "5. Do not drive the same joint by both position and effort at the same time.",
            "",
        ]
    )
    paths["md_latest"].write_text("\n".join(md_lines), encoding="utf-8")


def main():
    paths = _report_paths("arm_control_graph_attrs")
    report = {
        "generated_at": datetime.now().isoformat(timespec="seconds"),
        "status": "ok",
        "error": "",
        "matched_prim_count": 0,
        "matched_prims": [],
    }

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        report["status"] = "error"
        report["error"] = "No USD stage is open."
        _write_report(paths, report)
        return

    hits = []
    pseudo_root = stage.GetPseudoRoot()
    for prim in Usd.PrimRange(pseudo_root):
        matched_attrs = []
        for attr in prim.GetAttributes():
            if _attr_mentions(attr):
                matched_attrs.append((attr.GetName(), _safe_value(attr)))
        if matched_attrs:
            hits.append((prim, matched_attrs))

    report["matched_prim_count"] = len(hits)

    for prim, attrs in hits:
        schemas = prim.GetAppliedSchemas()
        prim_record = {
            "path": prim.GetPath().pathString,
            "type": prim.GetTypeName(),
            "schemas": list(schemas),
            "attributes": [],
        }
        for name, value in sorted(attrs):
            attr = prim.GetAttribute(name)
            prim_record["attributes"].append(
                {
                    "name": name,
                    "value": value,
                    "connections": _safe_connections(attr) if attr.IsValid() else [],
                }
            )
        report["matched_prims"].append(prim_record)

    _write_report(paths, report)


main()
