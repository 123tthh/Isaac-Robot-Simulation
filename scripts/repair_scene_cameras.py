"""Inspect and optionally repair the current scene's six-camera ROS 2 graph.

Run this file from Isaac Sim 5.1.0 Script Editor.  It writes a Markdown
diagnostic report and does not modify the USD unless APPLY_REPAIR is True.

Local documentation references:
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omniverse_usd/open_usd.md
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_modular_scripting.md
"""

from __future__ import annotations

import datetime as _dt
import asyncio
import os
import re
import shutil
from pathlib import Path

import omni.kit.app
import omni.usd
from pxr import Sdf


def _project_root() -> Path:
    configured = os.environ.get("ISAAC_OCS_PROJECT_ROOT")
    if configured:
        return Path(configured).expanduser().resolve()
    stage = omni.usd.get_context().get_stage()
    if stage is not None:
        layer_path = stage.GetRootLayer().realPath
        if layer_path:
            scene_path = Path(layer_path).resolve()
            if scene_path.name == "scene.usd" and scene_path.parent.name == "scenes":
                return scene_path.parents[2]
    raise RuntimeError(
        "Open assets/scenes/scene.usd first or set ISAAC_OCS_PROJECT_ROOT."
    )


PROJECT_ROOT = _project_root()
USD_PATH = PROJECT_ROOT / "assets/scenes/scene.usd"
REPORT_PATH = PROJECT_ROOT / "reports/scene_camera_repair.md"
BACKUP_DIR = PROJECT_ROOT / "backups/scene_camera_repair"
# Change to True only after reviewing the report generated with False.
APPLY_REPAIR = False

SIDES = ("H", "L", "R")
STREAMS = (
    ("head_rgb", "head_cam/color/image_raw", "rgb"),
    ("head_depth", "head_cam/depth/image_rect_raw", "depth"),
    ("left_rgb", "left_cam/color/image_raw", "rgb"),
    ("left_depth", "left_cam/depth/image_rect_raw", "depth"),
    ("right_rgb", "right_cam/color/image_raw", "rgb"),
    ("right_depth", "right_cam/depth/image_rect_raw", "depth"),
)
GRAPH_START = 'def OmniGraph "Camera_Publish_Graph"'
NODE_NAMES = (
    "on_playback_tick",
    "ros2_context",
    "cam_head_rgb",
    "cam_head_depth",
    "cam_left_rgb",
    "cam_left_depth",
    "cam_right_rgb",
    "cam_right_depth",
)


def _patch_graph(text: str) -> tuple[str, list[str]]:
    start = text.find(GRAPH_START)
    if start < 0:
        raise RuntimeError("Camera_Publish_Graph was not found")
    next_graph = text.find("        def OmniGraph ", start + len(GRAPH_START))
    end = next_graph if next_graph >= 0 else len(text)
    before, block, after = text[:start], text[start:end], text[end:]
    changed: list[str] = []
    for name in NODE_NAMES:
        plain = f'            def OmniGraphNode "{name}"\n            {{'
        replacement = (
            f'            def OmniGraphNode "{name}" (\n'
            '                prepend apiSchemas = ["NodeGraphNodeAPI"]\n'
            '            )\n            {'
        )
        if plain in block:
            block = block.replace(plain, replacement, 1)
            changed.append(name)
    return before + block + after, changed


def _has_schema(prim, schema: str) -> bool:
    return schema in set(prim.GetAppliedSchemas())


def _inspect(stage) -> tuple[list[str], list[str], list[str]]:
    lines: list[str] = []
    problems: list[str] = []
    missing: list[str] = []
    lines.append(f"- root_layer: `{stage.GetRootLayer().identifier}`")
    lines.append("")
    lines.append("## Camera prims")
    lines.append("")
    lines.append("| side | RSD455 | color camera | depth camera |")
    lines.append("|---|---|---|---|")
    for side in SIDES:
        root = f"/World/Robot/camera_{side}_link/Realsense_{side}/RSD455"
        root_prim = stage.GetPrimAtPath(root)
        color = stage.GetPrimAtPath(root + "/Camera_OmniVision_OV9782_Color")
        depth = stage.GetPrimAtPath(root + "/Camera_Pseudo_Depth")
        root_ok, color_ok, depth_ok = root_prim.IsValid(), color.IsValid(), depth.IsValid()
        lines.append(f"| {side} | `{root_ok}` | `{color_ok}` | `{depth_ok}` |")
        if not root_ok or not color_ok or not depth_ok:
            problems.append(f"camera prim missing for side {side}")

    lines.extend(("", "## Camera_Publish_Graph", "", "| node | NodeGraphNodeAPI |", "|---|---|"))
    graph = stage.GetPrimAtPath("/World/ActionGraphs/Camera_Publish_Graph")
    if not graph.IsValid():
        problems.append("Camera_Publish_Graph prim is missing")
    for name in NODE_NAMES:
        prim = stage.GetPrimAtPath(f"/World/ActionGraphs/Camera_Publish_Graph/{name}")
        ok = prim.IsValid() and _has_schema(prim, "NodeGraphNodeAPI")
        lines.append(f"| `{name}` | `{ok}` |")
        if not ok:
            missing.append(name)

    lines.extend(("", "## ROS 2 camera writer configuration", "", "| stream | topic | type | render product |", "|---|---|---|---|"))
    for stream, topic, kind in STREAMS:
        prim = stage.GetPrimAtPath(f"/World/ActionGraphs/Camera_Publish_Graph/cam_{stream}")
        actual_topic = str(prim.GetAttribute("inputs:topicName").Get() or "") if prim.IsValid() else ""
        product = str(prim.GetAttribute("inputs:renderProductPath").Get() or "") if prim.IsValid() else ""
        lines.append(f"| `{stream}` | `{actual_topic}` | `{kind}` | `{product}` |")
        if not prim.IsValid():
            problems.append(f"writer node missing: cam_{stream}")
        if actual_topic != topic:
            problems.append(f"topic mismatch for {stream}: expected {topic}, got {actual_topic}")
        if product:
            product_prim = stage.GetPrimAtPath(product)
            if not product_prim.IsValid():
                problems.append(f"render product missing for {stream}: {product}")
        else:
            problems.append(f"render product missing for {stream}")
    return lines, missing, problems


async def run() -> None:
    context = omni.usd.get_context()  # noqa: F821 - provided by Isaac Sim Script Editor
    stage = context.get_stage()
    if stage is None or not stage.GetRootLayer().identifier.endswith("workcell.usd"):
        context.open_stage(str(USD_PATH))
        for _ in range(120):
            await omni.kit.app.get_app().next_update_async()
        stage = context.get_stage()
    if stage is None or not stage.GetRootLayer().identifier.endswith("workcell.usd"):
        raise RuntimeError("workcell.usd did not finish loading; run the script again after opening it")
    report_lines = [
        "# workcell 相机 Script Editor 检查报告",
        "",
        f"生成时间：`{_dt.datetime.now().isoformat(timespec='seconds')}`",
        f"目标场景：`{USD_PATH}`",
        f"检查模式：`{'APPLY_REPAIR' if APPLY_REPAIR else 'READ_ONLY'}`",
        "",
    ]
    lines, missing, problems = _inspect(stage)
    report_lines.extend(lines)
    report_lines.extend(("", "## 结论", ""))
    if missing:
        report_lines.append("缺少 `NodeGraphNodeAPI` 的节点：" + ", ".join(f"`{x}`" for x in missing) + "。")
        if APPLY_REPAIR:
            BACKUP_DIR.mkdir(parents=True, exist_ok=True)
            backup = BACKUP_DIR / f"workcell.usd.before_script_editor_{_dt.datetime.now():%Y%m%d_%H%M%S}"
            shutil.copy2(USD_PATH, backup)
            source = Sdf.Layer.FindOrOpen(str(USD_PATH))
            if source is None:
                raise RuntimeError(f"cannot open {USD_PATH}")
            temp = Path("/tmp/scene_camera_repair.usda")
            source.Export(str(temp))
            patched, changed = _patch_graph(temp.read_text(encoding="utf-8"))
            temp.write_text(patched, encoding="utf-8")
            patched_layer = Sdf.Layer.FindOrOpen(str(temp))
            if patched_layer is None:
                raise RuntimeError("cannot reopen patched USDA")
            patched_layer.Export(str(USD_PATH))
            report_lines.append(f"已修复并保存；备份：`{backup}`。新增节点：{', '.join(changed)}。")
            report_lines.append("请在 Script Editor 执行 `omni.usd.get_context().open_stage(USD_PATH)` 或重新打开场景。")
        else:
            report_lines.append("当前为只读检查；确认后将 `APPLY_REPAIR = True` 再执行一次。")
    else:
        report_lines.append("八个相机图节点均含 `NodeGraphNodeAPI`，无需图架构修复。")
    if problems:
        report_lines.append("独立警告：" + "; ".join(problems))
    else:
        report_lines.append("相机 Prim、ROS 2 话题和 RenderProduct 静态配置均通过。")
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    print("\n".join(report_lines), flush=True)
    print(f"\nMarkdown report written to: {REPORT_PATH}", flush=True)


asyncio.ensure_future(run())
