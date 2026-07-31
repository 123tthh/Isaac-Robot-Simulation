"""
gripper_full_diagnosis_safe.py

Isaac Sim 5.1 PGIA gripper full diagnosis script.

安全版：
  - 不调用 app.update()
  - 不推进仿真
  - 不修改 USD
  - 不修改 Graph
  - 只读取和导出

输出：
  ~/ros2_log/gripper_full_diagnosis/gripper_full_diagnosis.md
  ~/ros2_log/gripper_full_diagnosis/gripper_full_diagnosis.json
  ~/ros2_log/gripper_full_diagnosis/gripper_runtime_trace.csv
  ~/ros2_log/gripper_full_diagnosis/gripper_test_commands.sh

建议：
  1. Stop 状态执行一次：检查 USD / Graph 静态配置
  2. Play 状态执行一次：检查 DynamicControl runtime DOF 状态
"""

import os
import json
import csv
import math
import traceback
from datetime import datetime

import omni.usd
import omni.graph.core as og

from pxr import UsdGeom, UsdPhysics, Gf


# ============================================================
# Config
# ============================================================

OUT_DIR = os.path.expanduser("~/ros2_log/gripper_full_diagnosis")
MD_PATH = os.path.join(OUT_DIR, "gripper_full_diagnosis.md")
JSON_PATH = os.path.join(OUT_DIR, "gripper_full_diagnosis.json")
CSV_PATH = os.path.join(OUT_DIR, "gripper_runtime_trace.csv")
CMD_PATH = os.path.join(OUT_DIR, "gripper_test_commands.sh")

GRAPH_PATH = "/World/ActionGraphs/Gripper_Control_Graph"

BASE_GAP = 0.0197

ROBOT_ART_CANDIDATES = [
    "/World/Robot/base_link",
    "/World/Robot",
    "/World/Robot/left_PGIA_base_link",
    "/World/Robot/right_PGIA_base_link",
]

JOINTS = {
    "left_j1_upper": "/World/Robot/joints/left_PGIA_joint1",
    "left_j2_lower": "/World/Robot/joints/left_PGIA_joint2",
    "right_j1_upper": "/World/Robot/joints/right_PGIA_joint1",
    "right_j2_lower": "/World/Robot/joints/right_PGIA_joint2",
}

JOINT_NAME_MAP = {
    "left_j1_upper": "left_PGIA_joint1",
    "left_j2_lower": "left_PGIA_joint2",
    "right_j1_upper": "right_PGIA_joint1",
    "right_j2_lower": "right_PGIA_joint2",
}

LINKS = {
    "left_base": "/World/Robot/left_PGIA_base_link",
    "left_link1_upper": "/World/Robot/left_PGIA_link1",
    "left_link2_lower": "/World/Robot/left_PGIA_link2",
    "right_base": "/World/Robot/right_PGIA_base_link",
    "right_link1_upper": "/World/Robot/right_PGIA_link1",
    "right_link2_lower": "/World/Robot/right_PGIA_link2",
}

GRAPH_NODES = {
    "sub_left": f"{GRAPH_PATH}/sub_left_gripper",
    "sub_right": f"{GRAPH_PATH}/sub_right_gripper",
    "script_left": f"{GRAPH_PATH}/script_left_gripper",
    "script_right": f"{GRAPH_PATH}/script_right_gripper",
    "artic_left": f"{GRAPH_PATH}/artic_left_gripper",
    "artic_right": f"{GRAPH_PATH}/artic_right_gripper",
}


# ============================================================
# Helpers
# ============================================================

def jsafe(x):
    if x is None:
        return None
    if isinstance(x, (str, int, float, bool)):
        return x
    if isinstance(x, dict):
        return {str(k): jsafe(v) for k, v in x.items()}
    if isinstance(x, (list, tuple)):
        return [jsafe(v) for v in x]
    try:
        return [jsafe(v) for v in x]
    except Exception:
        pass
    try:
        return str(x)
    except Exception:
        return repr(x)


def val(x):
    if isinstance(x, dict) and "value" in x:
        return x["value"]
    return x


def get_attr(prim, name, default=None):
    try:
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            return jsafe(attr.Get())
    except Exception:
        pass
    return default


def get_attr_stack(prim, name):
    out = {
        "value": get_attr(prim, name),
        "stack": [],
    }

    try:
        attr = prim.GetAttribute(name)
        if attr and attr.IsValid():
            for spec in attr.GetPropertyStack():
                out["stack"].append({
                    "layer": spec.layer.identifier if spec.layer else None,
                    "path": str(spec.path),
                    "value": jsafe(getattr(spec, "default", None)),
                })
    except Exception as e:
        out["stack_error"] = str(e)

    return out


def get_rel_targets(prim, rel_name):
    try:
        rel = prim.GetRelationship(rel_name)
        if rel and rel.IsValid():
            return [str(t) for t in rel.GetTargets()]
    except Exception:
        pass
    return []


def get_schemas(prim):
    try:
        return [str(s) for s in prim.GetAppliedSchemas()]
    except Exception:
        return []


def get_world_xform(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None

    try:
        cache = UsdGeom.XformCache()
        mat = cache.GetLocalToWorldTransform(prim)
        t = mat.ExtractTranslation()
        return {
            "translation": [float(t[0]), float(t[1]), float(t[2])],
            "matrix": [[float(mat[i][j]) for j in range(4)] for i in range(4)],
        }
    except Exception as e:
        return {"error": str(e)}


def get_bbox(stage, path):
    prim = stage.GetPrimAtPath(path)
    if not prim.IsValid():
        return None

    try:
        bbox_cache = UsdGeom.BBoxCache(
            UsdGeom.GetStageMetersPerUnit(stage),
            ["default", "render", "proxy"],
            useExtentsHint=True,
        )
        box = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
        mn = box.GetMin()
        mx = box.GetMax()
        return {
            "min": [float(mn[0]), float(mn[1]), float(mn[2])],
            "max": [float(mx[0]), float(mx[1]), float(mx[2])],
            "size": [
                float(mx[0] - mn[0]),
                float(mx[1] - mn[1]),
                float(mx[2] - mn[2]),
            ],
            "center": [
                float((mx[0] + mn[0]) * 0.5),
                float((mx[1] + mn[1]) * 0.5),
                float((mx[2] + mn[2]) * 0.5),
            ],
        }
    except Exception as e:
        return {"error": str(e)}


def axis_to_vec(axis_token):
    s = str(axis_token).upper()
    if s == "X":
        return Gf.Vec3d(1, 0, 0)
    if s == "Y":
        return Gf.Vec3d(0, 1, 0)
    if s == "Z":
        return Gf.Vec3d(0, 0, 1)
    return Gf.Vec3d(0, 0, 0)


def normalize_vec(v):
    l = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
    if l < 1e-12:
        return [0.0, 0.0, 0.0]
    return [float(v[0] / l), float(v[1] / l), float(v[2] / l)]


def transform_dir(mat, v):
    p0 = mat.Transform(Gf.Vec3d(0, 0, 0))
    p1 = mat.Transform(v)
    return p1 - p0


def get_upstream(attr):
    try:
        return [str(x) for x in attr.get_upstream_connections()]
    except Exception:
        return []


def get_downstream(attr):
    try:
        return [str(x) for x in attr.get_downstream_connections()]
    except Exception:
        return []


def safe_list_dir_methods(obj, patterns=None):
    out = []
    try:
        for name in dir(obj):
            if patterns is None or any(p in name for p in patterns):
                out.append(name)
    except Exception:
        pass
    return sorted(out)


# ============================================================
# USD Export
# ============================================================

def export_joint(stage, key, path):
    prim = stage.GetPrimAtPath(path)
    out = {
        "key": key,
        "path": path,
        "valid": bool(prim and prim.IsValid()),
    }

    if not out["valid"]:
        return out

    out["typeName"] = prim.GetTypeName()
    out["schemas"] = get_schemas(prim)

    joint = UsdPhysics.PrismaticJoint(prim)
    axis = joint.GetAxisAttr().Get()

    out["joint"] = {
        "axis": jsafe(axis),
        "lowerLimit": get_attr_stack(prim, "physics:lowerLimit"),
        "upperLimit": get_attr_stack(prim, "physics:upperLimit"),
        "jointEnabled": get_attr_stack(prim, "physics:jointEnabled"),
        "excludeFromArticulation": get_attr_stack(prim, "physics:excludeFromArticulation"),
        "collisionEnabled": get_attr_stack(prim, "physics:collisionEnabled"),
        "body0": get_rel_targets(prim, "physics:body0"),
        "body1": get_rel_targets(prim, "physics:body1"),
        "localPos0": get_attr_stack(prim, "physics:localPos0"),
        "localRot0": get_attr_stack(prim, "physics:localRot0"),
        "localPos1": get_attr_stack(prim, "physics:localPos1"),
        "localRot1": get_attr_stack(prim, "physics:localRot1"),
    }

    drive = UsdPhysics.DriveAPI.Get(prim, "linear")
    if drive:
        out["drive_linear"] = {
            "exists": True,
            "type": get_attr_stack(prim, "drive:linear:physics:type"),
            "stiffness": get_attr_stack(prim, "drive:linear:physics:stiffness"),
            "damping": get_attr_stack(prim, "drive:linear:physics:damping"),
            "maxForce": get_attr_stack(prim, "drive:linear:physics:maxForce"),
            "targetPosition": get_attr_stack(prim, "drive:linear:physics:targetPosition"),
            "targetVelocity": get_attr_stack(prim, "drive:linear:physics:targetVelocity"),
        }
    else:
        out["drive_linear"] = {"exists": False}

    suspicious = {}
    for attr in prim.GetAttributes():
        name = attr.GetName()
        low = name.lower()
        if any(k in low for k in ["mimic", "gear", "gearing", "limit", "drive"]):
            suspicious[name] = get_attr_stack(prim, name)
    out["suspicious_attrs"] = suspicious

    try:
        body0 = out["joint"]["body0"][0] if out["joint"]["body0"] else None
        if body0:
            b0 = stage.GetPrimAtPath(body0)
            cache = UsdGeom.XformCache()
            mat = cache.GetLocalToWorldTransform(b0)
            world_axis = normalize_vec(transform_dir(mat, axis_to_vec(axis)))
            out["world_axis_from_body0"] = {
                "axis": world_axis,
                "dot_gravity_down": float(world_axis[2] * -1.0),
            }
    except Exception as e:
        out["world_axis_error"] = str(e)

    return out


def export_link(stage, key, path):
    prim = stage.GetPrimAtPath(path)
    out = {
        "key": key,
        "path": path,
        "valid": bool(prim and prim.IsValid()),
    }

    if not out["valid"]:
        return out

    out["typeName"] = prim.GetTypeName()
    out["schemas"] = get_schemas(prim)
    out["world_xform"] = get_world_xform(stage, path)
    out["bbox"] = get_bbox(stage, path)

    fields = [
        "physics:rigidBodyEnabled",
        "physics:kinematicEnabled",
        "physics:mass",
        "physics:centerOfMass",
        "physics:diagonalInertia",
        "physxRigidBody:disableGravity",
        "physxRigidBody:linearDamping",
        "physxRigidBody:angularDamping",
        "physxRigidBody:solverPositionIterationCount",
        "physxRigidBody:solverVelocityIterationCount",
        "physxRigidBody:maxDepenetrationVelocity",
        "physics:collisionEnabled",
        "xformOp:translate",
        "xformOp:orient",
        "xformOp:rotateXYZ",
        "xformOp:scale",
        "xformOpOrder",
    ]

    out["attrs"] = {f: get_attr_stack(prim, f) for f in fields}

    children = []
    for child in prim.GetChildren():
        c = {
            "path": str(child.GetPath()),
            "typeName": child.GetTypeName(),
            "schemas": get_schemas(child),
            "bbox": get_bbox(stage, str(child.GetPath())),
            "attrs": {},
        }

        for f in [
            "physics:collisionEnabled",
            "physxCollision:contactOffset",
            "physxCollision:restOffset",
            "xformOp:translate",
            "xformOp:orient",
            "xformOp:rotateXYZ",
            "xformOp:scale",
            "xformOpOrder",
        ]:
            c["attrs"][f] = get_attr_stack(child, f)

        children.append(c)

    out["children"] = children
    return out


# ============================================================
# Graph Export
# ============================================================

def export_graph():
    out = {
        "graph_path": GRAPH_PATH,
        "graph_exists": False,
        "nodes": {},
        "connections_summary": [],
    }

    graph = og.get_graph_by_path(GRAPH_PATH)
    out["graph_exists"] = graph is not None

    for key, path in GRAPH_NODES.items():
        node = og.get_node_by_path(path)
        info = {
            "key": key,
            "path": path,
            "valid": bool(node and node.is_valid()),
            "attributes": {},
        }

        if node and node.is_valid():
            try:
                info["type_name"] = node.get_type_name()
            except Exception:
                info["type_name"] = None

            try:
                attrs = node.get_attributes()
            except Exception:
                attrs = []

            for attr in attrs:
                try:
                    name = attr.get_name()
                except Exception:
                    continue

                try:
                    value = attr.get()
                except Exception as e:
                    value = f"<GET_FAILED: {e}>"

                if isinstance(value, str) and len(value) > 6000:
                    value_short = value[:6000] + "\n...<TRUNCATED>"
                else:
                    value_short = value

                info["attributes"][name] = {
                    "value": jsafe(value_short),
                    "upstream": get_upstream(attr),
                    "downstream": get_downstream(attr),
                }

        out["nodes"][key] = info

    checks = [
        ("left jointNames", "script_left", "outputs:joint_names", "artic_left", "inputs:jointNames"),
        ("left positionCommand", "script_left", "outputs:position_cmds", "artic_left", "inputs:positionCommand"),
        ("left effortCommand", "script_left", "outputs:effort_cmds", "artic_left", "inputs:effortCommand"),
        ("left exec", "script_left", "outputs:execOut", "artic_left", "inputs:execIn"),
        ("left sub data", "sub_left", "outputs:data", "script_left", "inputs:input_double_array"),

        ("right jointNames", "script_right", "outputs:joint_names", "artic_right", "inputs:jointNames"),
        ("right positionCommand", "script_right", "outputs:position_cmds", "artic_right", "inputs:positionCommand"),
        ("right effortCommand", "script_right", "outputs:effort_cmds", "artic_right", "inputs:effortCommand"),
        ("right exec", "script_right", "outputs:execOut", "artic_right", "inputs:execIn"),
        ("right sub data", "sub_right", "outputs:data", "script_right", "inputs:input_double_array"),
    ]

    for label, sk, sa, dk, da in checks:
        src_path = GRAPH_NODES.get(sk)
        dst_path = GRAPH_NODES.get(dk)
        src_node = og.get_node_by_path(src_path) if src_path else None
        dst_node = og.get_node_by_path(dst_path) if dst_path else None

        item = {
            "label": label,
            "src": f"{src_path}.{sa}" if src_path else None,
            "dst": f"{dst_path}.{da}" if dst_path else None,
            "src_valid": False,
            "dst_valid": False,
            "dst_upstream": [],
            "connected": False,
        }

        if src_node and src_node.is_valid():
            src_attr = src_node.get_attribute(sa)
            item["src_valid"] = bool(src_attr and src_attr.is_valid())

        if dst_node and dst_node.is_valid():
            dst_attr = dst_node.get_attribute(da)
            item["dst_valid"] = bool(dst_attr and dst_attr.is_valid())
            if dst_attr and dst_attr.is_valid():
                ups = get_upstream(dst_attr)
                item["dst_upstream"] = ups
                item["connected"] = len(ups) > 0

        out["connections_summary"].append(item)

    return out


# ============================================================
# Runtime Export: no app.update()
# ============================================================

def export_runtime():
    out = {
        "available": False,
        "error": None,
        "articulation_path": None,
        "dofs": {},
        "methods": [],
        "runtime_trace_csv": CSV_PATH,
        "note": "Safe version: no app.update() is called.",
    }

    try:
        from omni.isaac.dynamic_control import _dynamic_control

        dc = _dynamic_control.acquire_dynamic_control_interface()

        out["methods"] = safe_list_dir_methods(
            dc,
            patterns=["get_dof", "set_dof", "get_articulation", "set_articulation"],
        )

        art = None
        art_path = None

        for p in ROBOT_ART_CANDIDATES:
            try:
                a = dc.get_articulation(p)
                if a:
                    art = a
                    art_path = p
                    break
            except Exception:
                pass

        if not art:
            out["error"] = "No articulation handle found. This is normal if simulation is stopped."
            write_empty_runtime_csv()
            return out

        out["available"] = True
        out["articulation_path"] = art_path

        dof_map = {}
        dof_count = dc.get_articulation_dof_count(art)

        for i in range(dof_count):
            dof = dc.get_articulation_dof(art, i)
            name = dc.get_dof_name(dof)
            dof_map[name] = dof

        row = {
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
        }

        for key, dof_name in JOINT_NAME_MAP.items():
            dof = dof_map.get(dof_name)
            info = {
                "dof_name": dof_name,
                "exists": dof is not None,
            }

            if dof:
                def sget(fn, *args):
                    try:
                        return jsafe(fn(*args))
                    except Exception as e:
                        return f"ERR: {e}"

                info.update({
                    "path": sget(dc.get_dof_path, dof),
                    "type": sget(dc.get_dof_type, dof),
                    "parent_body": sget(dc.get_dof_parent_body, dof),
                    "child_body": sget(dc.get_dof_child_body, dof),
                    "joint": sget(dc.get_dof_joint, dof),
                    "position": sget(dc.get_dof_position, dof),
                    "velocity": sget(dc.get_dof_velocity, dof),
                    "effort": sget(dc.get_dof_effort, dof),
                    "position_target": sget(dc.get_dof_position_target, dof),
                    "velocity_target": sget(dc.get_dof_velocity_target, dof),
                })

                try:
                    props = dc.get_dof_properties(dof)
                    pd = {}
                    for n in dir(props):
                        if n.startswith("_"):
                            continue
                        try:
                            v = getattr(props, n)
                            if not callable(v):
                                pd[n] = jsafe(v)
                        except Exception:
                            pass
                    info["properties"] = pd
                except Exception as e:
                    info["properties_error"] = str(e)

                try:
                    row[f"{dof_name}_pos"] = float(dc.get_dof_position(dof))
                    row[f"{dof_name}_vel"] = float(dc.get_dof_velocity(dof))
                    row[f"{dof_name}_eff"] = float(dc.get_dof_effort(dof))
                    row[f"{dof_name}_target"] = float(dc.get_dof_position_target(dof))
                except Exception:
                    pass

            out["dofs"][key] = info

        fields = sorted(row.keys())
        with open(CSV_PATH, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fields)
            writer.writeheader()
            writer.writerow(row)

    except Exception:
        out["error"] = traceback.format_exc()
        write_empty_runtime_csv()

    return out


def write_empty_runtime_csv():
    with open(CSV_PATH, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=["time", "note"])
        writer.writeheader()
        writer.writerow({
            "time": datetime.now().strftime("%H:%M:%S.%f")[:-3],
            "note": "No runtime articulation available or runtime export failed.",
        })


# ============================================================
# Analysis
# ============================================================

def compute_gap_analysis(data):
    out = {}

    try:
        j = data["usd"]["joints"]

        def lim(key, field):
            return float(val(j[key]["joint"][field]))

        l1_low = lim("left_j1_upper", "lowerLimit")
        l1_up = lim("left_j1_upper", "upperLimit")
        l2_low = lim("left_j2_lower", "lowerLimit")
        l2_up = lim("left_j2_lower", "upperLimit")

        r1_low = lim("right_j1_upper", "lowerLimit")
        r1_up = lim("right_j1_upper", "upperLimit")
        r2_low = lim("right_j2_lower", "lowerLimit")
        r2_up = lim("right_j2_lower", "upperLimit")

        out["left_min_gap_m"] = BASE_GAP + l1_low + l2_low
        out["left_max_gap_m"] = BASE_GAP + l1_up + l2_up
        out["right_min_gap_m"] = BASE_GAP + r1_low + r2_low
        out["right_max_gap_m"] = BASE_GAP + r1_up + r2_up

        out["left_min_gap_mm"] = out["left_min_gap_m"] * 1000.0
        out["left_max_gap_mm"] = out["left_max_gap_m"] * 1000.0
        out["right_min_gap_mm"] = out["right_min_gap_m"] * 1000.0
        out["right_max_gap_mm"] = out["right_max_gap_m"] * 1000.0

        out["limit_gap_symmetric"] = (
            abs(out["left_min_gap_m"] - out["right_min_gap_m"]) < 1e-6
            and abs(out["left_max_gap_m"] - out["right_max_gap_m"]) < 1e-6
        )

    except Exception as e:
        out["error"] = str(e)

    return out


def build_auto_analysis(data):
    analysis = {
        "critical": [],
        "warnings": [],
        "recommendations": [],
        "mode_guess": {},
    }

    joints = data["usd"]["joints"]
    graph = data["graph"]

    # Drive mode
    stiff_vals = []
    damp_vals = []

    for key, item in joints.items():
        d = item.get("drive_linear", {})
        stiff_vals.append(val(d.get("stiffness")))
        damp_vals.append(val(d.get("damping")))

    try:
        stiff_nums = [float(x) for x in stiff_vals if x is not None]
        damp_nums = [float(x) for x in damp_vals if x is not None]

        if stiff_nums and max(stiff_nums) == 0 and max(damp_nums) == 0:
            analysis["critical"].append(
                "All PGIA Drive stiffness/damping are 0. With vertical/near-vertical prismatic axes, lower fingers can fall under gravity in effort mode."
            )
            analysis["mode_guess"]["usd_drive"] = "pure_effort_or_free_joint"
        elif stiff_nums and max(stiff_nums) >= 1000:
            analysis["warnings"].append(
                "PGIA Drive stiffness is high. If effortCommand is connected, Drive may fight effort control."
            )
            analysis["mode_guess"]["usd_drive"] = "strong_position_drive"
        else:
            analysis["mode_guess"]["usd_drive"] = "weak_or_moderate_drive"

    except Exception as e:
        analysis["warnings"].append(f"Could not analyze drive mode: {e}")

    # Gravity projection
    for key, item in joints.items():
        wa = item.get("world_axis_from_body0", {})
        dot = wa.get("dot_gravity_down")
        try:
            if abs(float(dot)) > 0.7:
                analysis["warnings"].append(
                    f"{key} axis has strong gravity projection: dot_gravity_down={dot}. Gravity compensation or Drive holding is required."
                )
        except Exception:
            pass

    # Graph command mode
    pos_connected = []
    eff_connected = []

    for item in graph.get("connections_summary", []):
        label = item.get("label", "")
        ups = item.get("dst_upstream", [])
        if "positionCommand" in label and ups:
            pos_connected.append(label)
        if "effortCommand" in label and ups:
            eff_connected.append(label)

    if pos_connected and eff_connected:
        analysis["critical"].append(
            f"Both positionCommand and effortCommand have upstream connections: position={pos_connected}, effort={eff_connected}. This may cause control conflict."
        )
        analysis["mode_guess"]["graph_control"] = "mixed_position_and_effort"
    elif eff_connected:
        analysis["mode_guess"]["graph_control"] = "effortCommand"
        analysis["warnings"].append(
            "Graph appears to use effortCommand. Equal travel of joint1/joint2 is not guaranteed unless software PD/sync loop is used."
        )
    elif pos_connected:
        analysis["mode_guess"]["graph_control"] = "positionCommand"
        analysis["recommendations"].append(
            "Graph appears to use positionCommand. This is the cleanest path for strict joint1==joint2 equal travel."
        )
    else:
        analysis["mode_guess"]["graph_control"] = "no_position_or_effort_connection_detected"
        analysis["critical"].append(
            "No clear positionCommand or effortCommand upstream connection detected."
        )

    # Gap
    gap = data.get("gap_analysis", {})
    if gap.get("limit_gap_symmetric") is False:
        analysis["critical"].append("Left/right joint limits produce different min/max gap.")
    elif gap.get("limit_gap_symmetric") is True:
        analysis["recommendations"].append("Joint limits appear symmetric from gap calculation.")

    # Link transform differences
    links = data["usd"]["links"]
    try:
        l1z = links["left_link1_upper"]["world_xform"]["translation"][2]
        r1z = links["right_link1_upper"]["world_xform"]["translation"][2]
        l2z = links["left_link2_lower"]["world_xform"]["translation"][2]
        r2z = links["right_link2_lower"]["world_xform"]["translation"][2]

        if abs(float(l1z) - float(r1z)) > 0.03:
            analysis["warnings"].append(
                f"Upper link world Z differs significantly: left={l1z}, right={r1z}. Check xform/layer overrides."
            )

        if abs(float(l2z) - float(r2z)) > 0.03:
            analysis["warnings"].append(
                f"Lower link world Z differs significantly: left={l2z}, right={r2z}. Check xform/layer overrides."
            )
    except Exception:
        pass

    analysis["recommendations"].extend([
        "For strict equal travel, prefer positionCommand with identical [pos, pos] for joint1/joint2.",
        "For effort-based position control, use software PD: effort = Kp*(target-current) - Kd*velocity + gravity_comp.",
        "Do not judge control failure from repeated same command. Test close-open-close sequence.",
        "If FastDDS SHM errors appear, run: sudo rm -f /dev/shm/fastrtps_port* and consider export RMW_FASTRTPS_USE_SHM=0.",
    ])

    return analysis


# ============================================================
# Command file
# ============================================================

def write_test_commands():
    text = """#!/usr/bin/env bash
set -e

left_open() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [1]}"
}

left_close() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [0]}"
}

right_open() {
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [1]}"
}

right_close() {
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [0]}"
}

both_open() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [1]}" &
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [1]}" &
  wait
}

both_close() {
  ros2 topic pub --rate 10 --times 10 /left_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [0]}" &
  ros2 topic pub --rate 10 --times 10 /right_gripper_controller/commands \\
    std_msgs/msg/Float64MultiArray "{data: [0]}" &
  wait
}

cycle_test() {
  while true; do
    echo "[both close]"
    both_close
    sleep 2
    echo "[both open]"
    both_open
    sleep 2
  done
}

topic_info() {
  ros2 topic info /left_gripper_controller/commands -v
  ros2 topic info /right_gripper_controller/commands -v
}

clean_fastrtps_shm() {
  sudo rm -f /dev/shm/fastrtps_port*
}

case "$1" in
  left_open) left_open ;;
  left_close) left_close ;;
  right_open) right_open ;;
  right_close) right_close ;;
  both_open) both_open ;;
  both_close) both_close ;;
  cycle) cycle_test ;;
  topic_info) topic_info ;;
  clean_shm) clean_fastrtps_shm ;;
  *)
    echo "Usage: $0 {left_open|left_close|right_open|right_close|both_open|both_close|cycle|topic_info|clean_shm}"
    ;;
esac
"""
    with open(CMD_PATH, "w") as f:
        f.write(text)
    os.chmod(CMD_PATH, 0o755)


# ============================================================
# Markdown
# ============================================================

def md_list(lines, title, items):
    lines.append(f"## {title}")
    lines.append("")
    if not items:
        lines.append("- None")
    else:
        for x in items:
            lines.append(f"- {x}")
    lines.append("")


def write_markdown(data):
    lines = []

    lines.append("# PGIA Gripper Full Diagnosis")
    lines.append("")
    lines.append(f"- time: `{data['time']}`")
    lines.append(f"- stage root layer: `{data['stage']['root_layer']}`")
    lines.append(f"- edit target: `{data['stage']['edit_target']}`")
    lines.append(f"- output dir: `{OUT_DIR}`")
    lines.append(f"- json: `{JSON_PATH}`")
    lines.append(f"- runtime trace csv: `{CSV_PATH}`")
    lines.append(f"- test commands: `{CMD_PATH}`")
    lines.append("")

    lines.append("# 0. Background / Debug Context")
    lines.append("")
    lines.append("本报告用于集中排查 Isaac Sim PGIA 夹爪问题，包括：")
    lines.append("")
    lines.append("- USD joint / drive / link / xform / layer stack")
    lines.append("- Action Graph 连线")
    lines.append("- ScriptNode pin 与脚本内容摘要")
    lines.append("- ArticulationController 的 positionCommand / effortCommand 状态")
    lines.append("- DynamicControl runtime DOF 状态")
    lines.append("- 左右夹爪行程、gap、重力轴投影")
    lines.append("- ROS2 测试命令")
    lines.append("")
    lines.append("重要背景：")
    lines.append("")
    lines.append("- v3.8 纯 effort 同号施力不能严格保证 joint1/joint2 等行程。")
    lines.append("- right_PGIA_joint2 如果 prismatic 轴接近重力方向，stiffness=0 时容易下坠。")
    lines.append("- 直接写 DOF position 已验证右侧模型本体可以对称运动。")
    lines.append("- 严格等行程最干净的路线仍是 positionCommand + [pos, pos]。")
    lines.append("")

    lines.append("# 1. Auto Analysis")
    lines.append("")
    md_list(lines, "Critical", data["analysis"].get("critical", []))
    md_list(lines, "Warnings", data["analysis"].get("warnings", []))
    md_list(lines, "Recommendations", data["analysis"].get("recommendations", []))

    lines.append("## Mode Guess")
    lines.append("")
    for k, v in data["analysis"].get("mode_guess", {}).items():
        lines.append(f"- `{k}`: `{v}`")
    lines.append("")

    lines.append("# 2. Gap / Travel Calculation")
    lines.append("")
    gap = data.get("gap_analysis", {})
    for k, v in gap.items():
        lines.append(f"- `{k}`: `{v}`")
    lines.append("")
    lines.append("Expected relation:")
    lines.append("")
    lines.append("```text")
    lines.append("real_gap = BASE_GAP + joint1_position + joint2_position")
    lines.append("strict equal travel requires: joint1_position == joint2_position")
    lines.append("```")
    lines.append("")

    lines.append("# 3. USD Joint Details")
    lines.append("")
    for key, item in data["usd"]["joints"].items():
        lines.append(f"## {key}")
        lines.append(f"- path: `{item.get('path')}`")
        lines.append(f"- valid: `{item.get('valid')}`")
        lines.append(f"- typeName: `{item.get('typeName')}`")
        lines.append(f"- schemas: `{item.get('schemas')}`")
        lines.append("")

        lines.append("### joint")
        for k, v in item.get("joint", {}).items():
            lines.append(f"- `{k}`: `{v}`")
        lines.append("")

        lines.append("### drive_linear")
        for k, v in item.get("drive_linear", {}).items():
            lines.append(f"- `{k}`: `{v}`")
        lines.append("")

        lines.append("### world_axis_from_body0")
        lines.append(f"`{item.get('world_axis_from_body0')}`")
        lines.append("")

        lines.append("### suspicious_attrs")
        lines.append(f"`{item.get('suspicious_attrs')}`")
        lines.append("")

    lines.append("# 4. USD Link Details")
    lines.append("")
    for key, item in data["usd"]["links"].items():
        lines.append(f"## {key}")
        lines.append(f"- path: `{item.get('path')}`")
        lines.append(f"- valid: `{item.get('valid')}`")
        lines.append(f"- typeName: `{item.get('typeName')}`")
        lines.append(f"- schemas: `{item.get('schemas')}`")
        lines.append(f"- world_xform: `{item.get('world_xform')}`")
        lines.append(f"- bbox: `{item.get('bbox')}`")
        lines.append("")

        lines.append("### attrs")
        for k, v in item.get("attrs", {}).items():
            lines.append(f"- `{k}`: `{v}`")
        lines.append("")

        lines.append("### children")
        for c in item.get("children", []):
            lines.append(f"- `{c.get('path')}` type=`{c.get('typeName')}`")
            lines.append(f"  - schemas: `{c.get('schemas')}`")
            lines.append(f"  - bbox: `{c.get('bbox')}`")
            lines.append(f"  - attrs: `{c.get('attrs')}`")
        lines.append("")

    lines.append("# 5. Action Graph / Control Chain")
    lines.append("")
    g = data["graph"]
    lines.append(f"- graph path: `{g.get('graph_path')}`")
    lines.append(f"- graph exists: `{g.get('graph_exists')}`")
    lines.append("")

    lines.append("## Connection Summary")
    lines.append("")
    lines.append("| label | src | dst | connected | dst upstream |")
    lines.append("|---|---|---|---|---|")
    for c in g.get("connections_summary", []):
        lines.append(
            f"| {c.get('label')} | `{c.get('src')}` | `{c.get('dst')}` | "
            f"`{c.get('connected')}` | `{c.get('dst_upstream')}` |"
        )
    lines.append("")

    lines.append("## Graph Nodes")
    lines.append("")
    for key, node in g.get("nodes", {}).items():
        lines.append(f"### {key}")
        lines.append(f"- path: `{node.get('path')}`")
        lines.append(f"- valid: `{node.get('valid')}`")
        lines.append(f"- type_name: `{node.get('type_name')}`")
        lines.append("")

        for aname, ainfo in node.get("attributes", {}).items():
            show = False
            important_names = [
                "jointNames",
                "positionCommand",
                "effortCommand",
                "execIn",
                "execOut",
                "joint_names",
                "position_cmds",
                "effort_cmds",
                "input_double_array",
                "topicName",
                "messageName",
                "data",
                "script",
            ]

            if any(x.lower() in aname.lower() for x in important_names):
                show = True

            if not show:
                continue

            value = ainfo.get("value")
            if isinstance(value, str) and len(value) > 1200:
                value = value[:1200] + "\n...<TRUNCATED>"

            lines.append(f"- `{aname}`")
            lines.append(f"  - value: `{value}`")
            lines.append(f"  - upstream: `{ainfo.get('upstream')}`")
            lines.append(f"  - downstream: `{ainfo.get('downstream')}`")
        lines.append("")

    lines.append("# 6. Dynamic Control Runtime")
    lines.append("")
    rt = data["runtime"]
    lines.append(f"- available: `{rt.get('available')}`")
    lines.append(f"- articulation_path: `{rt.get('articulation_path')}`")
    lines.append(f"- error: `{rt.get('error')}`")
    lines.append(f"- note: `{rt.get('note')}`")
    lines.append("")

    lines.append("## DOF States")
    lines.append("")
    for key, info in rt.get("dofs", {}).items():
        lines.append(f"### {key}")
        for k, v in info.items():
            lines.append(f"- `{k}`: `{v}`")
        lines.append("")

    lines.append("# 7. ROS2 Commands")
    lines.append("")
    lines.append("Generated command file:")
    lines.append("")
    lines.append(f"```bash\n{CMD_PATH}\n```")
    lines.append("")
    lines.append("Examples:")
    lines.append("")
    lines.append("```bash")
    lines.append(f"{CMD_PATH} left_close")
    lines.append(f"{CMD_PATH} left_open")
    lines.append(f"{CMD_PATH} right_close")
    lines.append(f"{CMD_PATH} right_open")
    lines.append(f"{CMD_PATH} both_close")
    lines.append(f"{CMD_PATH} both_open")
    lines.append(f"{CMD_PATH} cycle")
    lines.append("```")
    lines.append("")

    lines.append("# 8. PD / Control Parameter Guidance")
    lines.append("")
    lines.append("## PositionCommand route")
    lines.append("")
    lines.append("Recommended for strict equal travel:")
    lines.append("")
    lines.append("```text")
    lines.append("Graph:")
    lines.append("  position_cmds -> positionCommand")
    lines.append("  effortCommand disconnected")
    lines.append("")
    lines.append("ScriptNode:")
    lines.append("  position_cmds = [pos, pos]")
    lines.append("")
    lines.append("USD Drive:")
    lines.append("  stiffness = 1000~3000")
    lines.append("  damping   = 100~300")
    lines.append("  maxForce  = 300~500")
    lines.append("```")
    lines.append("")

    lines.append("## Effort-PD route")
    lines.append("")
    lines.append("Only use if force-level control is required:")
    lines.append("")
    lines.append("```text")
    lines.append("Graph:")
    lines.append("  effort_cmds -> effortCommand")
    lines.append("  positionCommand disconnected")
    lines.append("")
    lines.append("USD Drive:")
    lines.append("  stiffness = 0 or very small")
    lines.append("  damping   = 0 or very small")
    lines.append("")
    lines.append("Software PD:")
    lines.append("  effort = Kp*(target-current) - Kd*filtered_velocity + gravity_comp")
    lines.append("")
    lines.append("Suggested initial parameters:")
    lines.append("  KP = 250~500")
    lines.append("  KD = 45~80")
    lines.append("  EFFORT_MAX = 30~50")
    lines.append("  EMA_ALPHA = 0.10~0.20")
    lines.append("  J2_GRAV_COMP = -5.0~-10.0")
    lines.append("  CLOSE_STEP = 0.0001~0.0003")
    lines.append("  VEL_THRESH = 0.003")
    lines.append("  CONTACT_CONFIRM = 8~15")
    lines.append("```")
    lines.append("")

    lines.append("# 9. Checklist")
    lines.append("")
    checklist = [
        "Only one command mode should be active: positionCommand OR effortCommand, not both.",
        "If using effortCommand, same effort does not guarantee equal travel.",
        "If strict equal travel is required, prefer positionCommand or Mimic/gear coupling.",
        "If lower finger falls with no tray, check world_axis dot_gravity_down and Drive stiffness/damping.",
        "If first ROS command works but later commands fail, check FastDDS SHM and resource pressure.",
        "If GUI shows grey jointNames, check upstream connection instead of editing grey field directly.",
        "If left/right behavior differs, compare layer stack for xformOp, drive, mass, disableGravity.",
        "Do not run app.update() inside this diagnosis script in Isaac Sim 5.1.",
    ]
    for x in checklist:
        lines.append(f"- {x}")
    lines.append("")

    with open(MD_PATH, "w") as f:
        f.write("\n".join(lines))


# ============================================================
# Main
# ============================================================

def main():
    os.makedirs(OUT_DIR, exist_ok=True)

    stage = omni.usd.get_context().get_stage()
    if stage is None:
        raise RuntimeError("No USD stage found")

    try:
        root_layer = stage.GetRootLayer().identifier
    except Exception:
        root_layer = None

    try:
        edit_target = stage.GetEditTarget().GetLayer().identifier
    except Exception:
        edit_target = None

    data = {
        "time": str(datetime.now()),
        "stage": {
            "root_layer": root_layer,
            "edit_target": edit_target,
        },
        "usd": {
            "joints": {},
            "links": {},
        },
        "graph": {},
        "runtime": {},
        "gap_analysis": {},
        "analysis": {},
    }

    print("=" * 80)
    print("PGIA gripper full diagnosis SAFE version")
    print("=" * 80)
    print("No app.update() will be called.")
    print("Output dir:", OUT_DIR)
    print("=" * 80)

    print("[1/7] Export USD joints")
    for key, path in JOINTS.items():
        data["usd"]["joints"][key] = export_joint(stage, key, path)

    print("[2/7] Export USD links")
    for key, path in LINKS.items():
        data["usd"]["links"][key] = export_link(stage, key, path)

    print("[3/7] Export Action Graph")
    data["graph"] = export_graph()

    print("[4/7] Export DynamicControl runtime snapshot")
    data["runtime"] = export_runtime()

    print("[5/7] Compute gap analysis")
    data["gap_analysis"] = compute_gap_analysis(data)

    print("[6/7] Build auto analysis")
    data["analysis"] = build_auto_analysis(data)

    print("[7/7] Write outputs")
    write_test_commands()

    safe_data = jsafe(data)

    with open(JSON_PATH, "w") as f:
        json.dump(safe_data, f, indent=2, ensure_ascii=False)

    write_markdown(safe_data)

    print("")
    print("=" * 80)
    print("DONE")
    print("=" * 80)
    print("MD  :", MD_PATH)
    print("JSON:", JSON_PATH)
    print("CSV :", CSV_PATH)
    print("CMD :", CMD_PATH)
    print("")
    print("View summary:")
    print(f"sed -n '1,220p' {MD_PATH}")
    print("")
    print("Run tests from Ubuntu terminal, not Script Editor:")
    print(f"{CMD_PATH} both_close")
    print(f"{CMD_PATH} both_open")
    print("=" * 80)


main()
