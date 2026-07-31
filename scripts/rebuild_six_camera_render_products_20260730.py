"""Rebuild and bind the six ROS 2 camera RenderProducts for this Kit session.

Run this file from the Isaac Sim 5.1.0 Script Editor while the timeline is
stopped.  It can also be passed to Isaac Sim with ``--exec``; in that mode it
waits for the canonical project stage to finish loading before rebuilding.

The RenderProducts and CameraHelper binding overrides are session state.  The
actual paths returned by ``rep.create.render_product`` are always used, and no
new session RenderProduct path is saved into the root USD.

Local documentation references:
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/development_tools/omniverse_script_editor.md
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.core.utils/docs/api.md
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_amr_navigation.md
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_getting_started.md
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/replicator_tutorials/tutorial_replicator_modular_scripting.md
  /home/gtk/ai_docs/docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_camera_publishing.md
"""

from __future__ import annotations

import asyncio
import builtins
import datetime as dt
import json
import os
import time
import traceback
from pathlib import Path

import omni.graph.core as og
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
import omni.usd
import isaacsim.core.utils.stage as stage_utils
from pxr import Sdf


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

RESOLUTION = (640, 480)

RENDER_ROOT = "/Render/OmniverseKit/HydraTextures"
GRAPH_ROOT = "/World/ActionGraphs/Camera_Publish_Graph"

CONFIGS = [
    {
        "name": "head_rgb",
        "camera": (
            "/World/Robot/camera_H_link/Realsense_H/RSD455/"
            "Camera_OmniVision_OV9782_Color"
        ),
        "helper": f"{GRAPH_ROOT}/cam_head_rgb",
    },
    {
        "name": "head_depth",
        "camera": (
            "/World/Robot/camera_H_link/Realsense_H/RSD455/"
            "Camera_Pseudo_Depth"
        ),
        "helper": f"{GRAPH_ROOT}/cam_head_depth",
    },
    {
        "name": "left_rgb",
        "camera": (
            "/World/Robot/camera_L_link/Realsense_L/RSD455/"
            "Camera_OmniVision_OV9782_Color"
        ),
        "helper": f"{GRAPH_ROOT}/cam_left_rgb",
    },
    {
        "name": "left_depth",
        "camera": (
            "/World/Robot/camera_L_link/Realsense_L/RSD455/"
            "Camera_Pseudo_Depth"
        ),
        "helper": f"{GRAPH_ROOT}/cam_left_depth",
    },
    {
        "name": "right_rgb",
        "camera": (
            "/World/Robot/camera_R_link/Realsense_R/RSD455/"
            "Camera_OmniVision_OV9782_Color"
        ),
        "helper": f"{GRAPH_ROOT}/cam_right_rgb",
    },
    {
        "name": "right_depth",
        "camera": (
            "/World/Robot/camera_R_link/Realsense_R/RSD455/"
            "Camera_Pseudo_Depth"
        ),
        "helper": f"{GRAPH_ROOT}/cam_right_depth",
    },
]

BASE_NAMES = {item["name"] for item in CONFIGS}

# Keep ReplicatorItems alive until the current Isaac Sim process exits or this
# script explicitly replaces them.
HANDLE_STORE = "_ocs_six_camera_render_products"

PROJECT_ROOT = Path(
    os.environ.get("ISAAC_OCS_PROJECT_ROOT", "/home/gtk/isaac_ocs_project")
).expanduser().resolve()
EXPECTED_SCENE = Path(
    os.environ.get(
        "ISAAC_USD_PATH",
        str(PROJECT_ROOT / "assets/scenes/scene.usd"),
    )
).expanduser().resolve()
REPORT_PATH = PROJECT_ROOT / "outputs/camera_session_rebuild/latest.json"
STAGE_WAIT_SECONDS = float(os.environ.get("OCS_CAMERA_STAGE_WAIT_SECONDS", "120"))
AUTO_PLAY_AFTER_REBUILD = (
    os.environ.get("OCS_CAMERA_REBUILD_AUTOPLAY", "0") == "1"
)


# ---------------------------------------------------------------------------
# OmniGraph utilities
# ---------------------------------------------------------------------------


def get_node(node_path: str):
    node = og.get_node_by_path(node_path)
    if not node:
        raise RuntimeError(f"找不到 OmniGraph 节点：{node_path}")
    return node


def get_node_attr(node_path: str, attr_name: str):
    node = get_node(node_path)
    attr = node.get_attribute(attr_name)
    if not attr:
        raise RuntimeError(f"找不到 OmniGraph 属性：{node_path}.{attr_name}")
    return attr


def set_attr(attr, value) -> None:
    try:
        attr.set(value)
    except Exception:
        og.Controller.set(attr, value)


# ---------------------------------------------------------------------------
# RenderProduct utilities
# ---------------------------------------------------------------------------


def is_target_render_product_path(path: str) -> bool:
    if not path.startswith(RENDER_ROOT + "/"):
        return False

    leaf = path.rsplit("/", 1)[-1]
    return any(
        leaf == base_name or leaf.startswith(base_name + "_")
        for base_name in BASE_NAMES
    )


def delete_spec_from_layer(layer, path: str) -> bool:
    sdf_path = Sdf.Path(path)
    try:
        spec = layer.GetPrimAtPath(sdf_path)
    except Exception:
        spec = None

    if spec is None:
        return False

    edit = Sdf.BatchNamespaceEdit()
    edit.Add(Sdf.NamespaceEdit.Remove(sdf_path))
    if not layer.Apply(edit):
        raise RuntimeError(
            f"无法从 Layer 删除 PrimSpec：{layer.identifier} {sdf_path}"
        )
    return True


def collect_target_render_products(stage) -> list[str]:
    paths: set[str] = set()

    # Collect existing camera RenderProducts. Replicator_* is intentionally
    # outside the target-name predicate and is never deleted by this script.
    for prim in stage.Traverse():
        path = prim.GetPath().pathString
        if prim.GetTypeName() == "RenderProduct" and is_target_render_product_path(path):
            paths.add(path)

    # Collect stale CameraHelper references as well. This is done before the
    # helpers are cleared so nonexistent old paths are still visible in logs.
    for config in CONFIGS:
        try:
            attr = get_node_attr(config["helper"], "inputs:renderProductPath")
            path = str(attr.get() or "")
            if is_target_render_product_path(path):
                paths.add(path)
        except Exception:
            pass

    return sorted(paths)


def destroy_previous_handles() -> None:
    old_handles = getattr(builtins, HANDLE_STORE, {})
    if not old_handles:
        print("  没有发现本会话保存的旧句柄")
        return

    for name, handle in list(old_handles.items()):
        try:
            handle.destroy()
            print("  destroyed handle:", name)
        except Exception as exc:
            print("  destroy handle failed:", name, exc)

    setattr(builtins, HANDLE_STORE, {})


def _root_real_path(stage) -> Path | None:
    real_path = str(stage.GetRootLayer().realPath or "")
    if not real_path:
        return None
    return Path(real_path).expanduser().resolve()


async def wait_for_expected_stage():
    context = omni.usd.get_context()
    stage = context.get_stage()

    if stage is None or _root_real_path(stage) != EXPECTED_SCENE:
        print("Opening target stage:", EXPECTED_SCENE)
        open_result = await context.open_stage_async(str(EXPECTED_SCENE))
        print("open_stage_async result:", open_result)

    deadline = time.monotonic() + STAGE_WAIT_SECONDS
    while time.monotonic() < deadline:
        stage = context.get_stage()
        if (
            stage is not None
            and not stage_utils.is_stage_loading()
            and _root_real_path(stage) == EXPECTED_SCENE
            and stage.GetPrimAtPath(GRAPH_ROOT).IsValid()
        ):
            return stage
        await omni.kit.app.get_app().next_update_async()

    stage = context.get_stage()
    current = _root_real_path(stage) if stage is not None else None
    raise RuntimeError(
        "目标场景未在等待时间内加载完成："
        f"expected={EXPECTED_SCENE}, current={current}"
    )


# ---------------------------------------------------------------------------
# Main repair
# ---------------------------------------------------------------------------


async def rebuild_six_camera_render_products() -> dict:
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("当前没有打开 USD Stage")

    root_layer = stage.GetRootLayer()
    session_layer = stage.GetSessionLayer()
    original_edit_target = stage.GetEditTarget()

    timeline = omni.timeline.get_timeline_interface()
    was_playing = timeline.is_playing()

    print("=" * 80)
    print("Rebuild Six Camera RenderProducts")
    print("=" * 80)
    print("Root USD:", root_layer.realPath)
    print("Session layer:", session_layer.identifier)
    print("Resolution:", RESOLUTION)
    print("Timeline was playing:", was_playing)

    if was_playing:
        timeline.stop()
        print("Timeline stopped")

    created: dict[str, str] = {}
    failures: dict[str, str] = {}
    removed_from_session: list[str] = []
    removed_from_root: list[str] = []
    target_paths = collect_target_render_products(stage)

    try:
        print("\n[1/7] Disable helpers and clear old bindings")
        stage.SetEditTarget(session_layer)

        for config in CONFIGS:
            helper = config["helper"]
            enabled_attr = get_node_attr(helper, "inputs:enabled")
            path_attr = get_node_attr(helper, "inputs:renderProductPath")
            set_attr(enabled_attr, False)
            set_attr(path_attr, "")
            print("  cleared:", helper)

        await omni.kit.app.get_app().next_update_async()

        print("\n[2/7] Destroy previous session handles")
        destroy_previous_handles()
        await omni.kit.app.get_app().next_update_async()

        print("\n[3/7] Remove old camera RenderProducts")
        if not target_paths:
            print("  No old camera RenderProducts found")

        for path in target_paths:
            removed = False

            if delete_spec_from_layer(session_layer, path):
                print("  removed from session:", path)
                removed_from_session.append(path)
                removed = True

            if delete_spec_from_layer(root_layer, path):
                print("  removed from root:", path)
                removed_from_root.append(path)
                removed = True

            if not removed:
                print("  stale helper path only:", path)

        # Save only deletion of stale, camera-specific RenderProduct specs.
        # Session bindings created below are never saved into the root layer.
        if removed_from_root:
            root_layer.Save()
            print("  Root USD saved after stale camera RenderProduct removal")

        # Allow Hydra/SyntheticData two frames to release the old resources.
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        print("\n[4/7] Validate camera prims")
        for config in CONFIGS:
            camera_path = config["camera"]
            camera_prim = stage.GetPrimAtPath(camera_path)

            if not camera_prim.IsValid():
                raise RuntimeError(f"Camera Prim 不存在：{camera_path}")

            if camera_prim.GetTypeName() != "Camera":
                raise RuntimeError(
                    f"目标 Prim 不是 Camera：{camera_path}, "
                    f"type={camera_prim.GetTypeName()}"
                )

            print("  OK:", camera_path)

        print("\n[5/7] Create and bind RenderProducts")
        stage.SetEditTarget(session_layer)
        handles = {}

        for config in CONFIGS:
            name = config["name"]
            camera_path = config["camera"]
            helper_path = config["helper"]

            print(f"\n  [{name}]")
            print("    camera:", camera_path)

            try:
                render_product = rep.create.render_product(
                    camera_path,
                    RESOLUTION,
                    force_new=True,
                    name=name,
                )

                # The actual return path is authoritative because a duplicate
                # name is suffixed by Replicator (for example, _01).
                rp_path = str(render_product.path)
                rp_prim = stage.GetPrimAtPath(rp_path)

                if not rp_prim.IsValid():
                    raise RuntimeError(
                        f"创建后的 RenderProduct Prim 无效：{rp_path}"
                    )

                if rp_prim.GetTypeName() != "RenderProduct":
                    raise RuntimeError(
                        "Prim 类型不是 RenderProduct："
                        f"{rp_path}, type={rp_prim.GetTypeName()}"
                    )

                camera_targets = [
                    str(target)
                    for target in rp_prim.GetRelationship("camera").GetTargets()
                ]

                if camera_path not in camera_targets:
                    raise RuntimeError(
                        "camera relationship 不正确："
                        f"{camera_targets}"
                    )

                path_attr = get_node_attr(
                    helper_path,
                    "inputs:renderProductPath",
                )
                set_attr(path_attr, rp_path)

                bound_path = str(path_attr.get() or "")
                if bound_path != rp_path:
                    raise RuntimeError(
                        f"绑定回读失败：{bound_path} != {rp_path}"
                    )

                handles[name] = render_product
                created[name] = rp_path

                print("    render product:", rp_path)
                print("    helper:", helper_path)
                print("    camera relationship:", camera_targets)

            except Exception as exc:
                failures[name] = str(exc)
                print("    ERROR:", exc)

        setattr(builtins, HANDLE_STORE, handles)

        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        print("\n[6/7] Enable successful helpers")
        for config in CONFIGS:
            name = config["name"]
            helper = config["helper"]
            enabled_attr = get_node_attr(helper, "inputs:enabled")

            if name in created:
                set_attr(enabled_attr, True)
                print("  enabled:", helper)
            else:
                set_attr(enabled_attr, False)
                print("  kept disabled:", helper)

        await omni.kit.app.get_app().next_update_async()

    finally:
        stage.SetEditTarget(original_edit_target)

    print("\n[7/7] Final result")
    print("=" * 80)

    for config in CONFIGS:
        name = config["name"]

        if name in created:
            print(f"[OK] {name}")
            print("    ", created[name])
        else:
            print(f"[FAILED] {name}")
            print("        ", failures.get(name, "unknown error"))

    print("=" * 80)

    result = {
        "generated_at": dt.datetime.now().astimezone().isoformat(
            timespec="seconds"
        ),
        "status": "ok" if not failures and len(created) == 6 else "failed",
        "root_usd": str(root_layer.realPath),
        "session_layer": session_layer.identifier,
        "resolution": list(RESOLUTION),
        "created": created,
        "failures": failures,
        "removed_from_session": removed_from_session,
        "removed_from_root": removed_from_root,
        "handle_store": HANDLE_STORE,
        "timeline_was_playing": was_playing,
        "auto_play_after_rebuild": AUTO_PLAY_AFTER_REBUILD,
    }

    if failures:
        print("部分创建失败，失败节点保持 disabled。")
        return result

    print("六路 RenderProduct 已重新创建并绑定。")

    if was_playing or AUTO_PLAY_AFTER_REBUILD:
        timeline.play()
        print("Timeline resumed")
    else:
        print("现在点击 Play，然后检查 ROS 2 topics。")

    return result


def write_report(result: dict) -> None:
    REPORT_PATH.parent.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(
        json.dumps(result, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print("Session camera report:", REPORT_PATH)


async def run_when_stage_ready() -> None:
    try:
        await wait_for_expected_stage()
        result = await rebuild_six_camera_render_products()
        write_report(result)
    except Exception as exc:
        result = {
            "generated_at": dt.datetime.now().astimezone().isoformat(
                timespec="seconds"
            ),
            "status": "failed",
            "root_usd": str(EXPECTED_SCENE),
            "created": {},
            "failures": {"startup": str(exc)},
            "traceback": traceback.format_exc(),
        }
        write_report(result)
        traceback.print_exc()


# Script Editor and Kit --exec both provide a running asyncio event loop.
asyncio.ensure_future(run_when_stage_ready())
