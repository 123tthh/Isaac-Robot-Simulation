"""
PGIA 夹爪位控 v4.2 — 统一版 (左右共用)
================================================================
参考本地文档:
  https://docs.isaacsim.omniverse.nvidia.com/5.1.0/
    ros2_tutorials/tutorial_ros2_manipulation.md

根因修复:
  诊断显示 USD Drive stiffness=2000 (强位控), 但 Graph 连 effortCommand
  → Position Drive 压制 effort, 夹爪不响应
  v4.2 改用 positionCommand, 配合 stiffness=2000 的强位控

控制策略 (满足需求):
  - 任意托盘高度闭合: target 逐帧减小, Drive 推动闭合
    碰到托盘 joint 停住, Drive 弹簧力自然增大 (力位检测)
  - 无托盘: target 一直减到 MIN_POS, 走到最小行程
  - 强位控 stiffness=2000 自动补偿竖直轴重力, 不下坠

ROS2 指令:
  data[0] > 0   → 张开 (走到 MAX_TRAVEL)
  data[0] <= 0  → 闭合 (走到接触或 MIN_POS)
  data[0] = 具体正值 (如 0.08) → 可选: 直接定位到该 gap (高级)

接触检测 (力位):
  闭合中 joint 速度趋零 + 位置不再变化 = 碰到托盘 → HOLD 锁定
  (此时 Drive 弹簧力 = stiffness × (target-current) 提供夹持力)

Graph 连线: position_cmds → positionCommand (不是 effortCommand!)
USD Drive: stiffness=2000, damping=200, maxForce=500

左右共用: 自动检测 left/right
================================================================
"""

import os
import math
import builtins
from datetime import datetime
from pathlib import Path

import numpy as np
import omni.graph.core as og
import carb

try:
    from omni.isaac.dynamic_control import _dynamic_control
except Exception:
    _dynamic_control = None


# ============================================================
# 状态常量
# ============================================================
ST_INIT = -1; ST_IDLE = 0; ST_OPEN = 1; ST_CLOSE = 2; ST_HOLD = 3
ST_NAME = {-1: "INIT", 0: "IDLE", 1: "OPEN", 2: "CLOSE", 3: "HOLD"}


# ============================================================
# 独立状态存储 (防全局污染)
# ============================================================
if not hasattr(builtins, "_PGIA_V42_STATE"):
    builtins._PGIA_V42_STATE = {}


# ============================================================
# 配置
# ============================================================
def _cfg(side):
    c = {
        "side": side,
        "joint1": f"{side}_PGIA_joint1",
        "joint2": f"{side}_PGIA_joint2",

        # ===== 几何 (来自 diagnosis) =====
        "BASE_GAP": 0.0197,
        "USD_LOWER": 0.004,          # diagnosis: lowerLimit=0.004
        "USD_UPPER": 0.12,           # diagnosis: upperLimit=0.12

        # ===== 行程 =====
        # 需求: 无托盘走到最小行程
        # MIN_POS = USD_LOWER 附近, 留一点余量
        "MIN_POS": 0.005,            # 单 joint 最小目标 (m)
        # gap_min = 0.0197 + 2×0.005 = 29.7mm

        # 需求: 张开最大行程
        # diagnosis max_gap=259.7mm @ pos=0.12
        "MAX_POS": 0.118,            # 单 joint 最大目标 (m), 略低于 0.12
        # gap_max = 0.0197 + 2×0.118 = 255.7mm

        # ===== 闭合 =====
        "CLOSE_STEP": 0.0005,        # 每帧 target 减小 (m), 60Hz→30mm/s
        # 调参: 0.0003=慢 0.0005=中 0.001=快

        # ===== 张开 =====
        "OPEN_STEP": 0.0008,         # 每帧 target 增大 (m), 60Hz→48mm/s

        # ===== 接触检测 (力位) =====
        # Position Drive 模式下 DC effort 有值
        # 用 速度趋零 + 位置稳定 判断接触
        "VEL_THRESH": 0.003,         # 速度阈值 (m/s)
        "POS_STABLE": 0.0006,        # 位置帧间变化 (m)
        "CONTACT_CONFIRM": 8,        # 连续确认帧
        # 接触后 target 与实际位置的差就是夹持预紧
        "HOLD_PRELOAD": 0.003,       # 预紧 (m): target = contact - PRELOAD

        # ===== 脱落 =====
        "SLIP_VEL": 0.02,

        # ===== 预热 =====
        "WARMUP_FRAMES": 8,

        # ===== 初始 =====
        # 需求: 初始夹爪含托盘 → 初始就闭合夹持
        "INIT_CLOSE": True,          # True=初始进闭合; False=初始张开

        "LOG_DIR": os.environ.get("ROS2_LOG_DIR") or str(
            (Path(globals()["__file__"]).resolve().parents[3]
             if "__file__" in globals()
             else Path(os.environ.get("ISAAC_OCS_PROJECT_ROOT", os.getcwd()))) / "data/ros2_log"
        ),
        "LOG_INTERVAL": 30,
    }
    c["LOG_FILE"] = f'{c["LOG_DIR"]}/{side}_gripper_v42.log'
    return c


# ============================================================
# 节点身份
# ============================================================
def _node_path(db):
    for an in ["abi_node", "node"]:
        try:
            n = getattr(db, an)
            for mn in ["get_prim_path", "get_path"]:
                try:
                    p = str(getattr(n, mn)())
                    if p and "script_" in p: return p
                except: pass
        except: pass
    return "unknown"

def _side(db):
    p = _node_path(db).lower()
    return "right" if "right" in p else "left"

def _state(db):
    path = _node_path(db); side = _side(db)
    key = f"{path}:{side}"
    store = builtins._PGIA_V42_STATE
    if key not in store:
        store[key] = {
            "path": path, "side": side,
            "state": ST_INIT, "prev": ST_INIT,
            "frame": 0, "ccnt": 0,
            "target": 0.02, "hold_pos": 0.02,
            "pp1": 0.0, "pp2": 0.0,
            "dc": None, "art": 0, "dof": {}, "dc_retry": -9999,
        }
    return store[key]


# ============================================================
# 工具
# ============================================================
def _sf(x, d=0.0):
    try:
        if x is None: return d
        y = float(x)
        return d if (math.isnan(y) or math.isinf(y)) else y
    except: return d

def _gap(c, j1, j2): return c["BASE_GAP"] + j1 + j2

def _log(st, c, msg, force=False):
    if not force and (st["state"] == st["prev"]) and (st["frame"] % c["LOG_INTERVAL"] != 0): return
    line = f"{datetime.now().strftime('%H:%M:%S.%f')[:-3]} f={st['frame']} [{c['side']}] {msg}\n"
    try:
        os.makedirs(c["LOG_DIR"], exist_ok=True)
        with open(c["LOG_FILE"], "a") as f: f.write(line)
    except: pass
    carb.log_info(f"[{c['side']}_v42] {msg}")


# ============================================================
# DC
# ============================================================
def _init_dc(st, c):
    if st["dc"] and st["art"] and c["joint1"] in st["dof"] and c["joint2"] in st["dof"]:
        return True
    if (st["frame"] - st["dc_retry"]) < 30: return False
    st["dc_retry"] = st["frame"]
    if not _dynamic_control:
        _log(st, c, "DC unavailable", True); return False
    try: st["dc"] = _dynamic_control.acquire_dynamic_control_interface()
    except Exception as e:
        _log(st, c, f"DC fail: {e}", True); st["dc"] = None; return False
    st["art"] = 0
    for p in ["/World/Robot/base_link", "/World/Robot"]:
        try:
            h = st["dc"].get_articulation(p)
            if h: st["art"] = h; _log(st, c, f"Art: {p}", True); break
        except: pass
    if not st["art"]: _log(st, c, "Art not found", True); return False
    try:
        n = st["dc"].get_articulation_dof_count(st["art"]); st["dof"] = {}
        for i in range(n):
            d = st["dc"].get_articulation_dof(st["art"], i)
            st["dof"][st["dc"].get_dof_name(d)] = i
        _log(st, c, f"DOF={n} j1=[{st['dof'].get(c['joint1'])}] j2=[{st['dof'].get(c['joint2'])}]", True)
        return c["joint1"] in st["dof"] and c["joint2"] in st["dof"]
    except Exception as e:
        _log(st, c, f"DOF fail: {e}", True); return False

def _read2(st, c):
    if not st["dc"] or not st["art"]: return 0.0, 0.0, 0.0, 0.0, False
    try:
        s = st["dc"].get_articulation_dof_states(st["art"], _dynamic_control.STATE_ALL)
        if s is None: return 0.0, 0.0, 0.0, 0.0, False
        i1 = st["dof"].get(c["joint1"]); i2 = st["dof"].get(c["joint2"])
        if i1 is None or i2 is None: return 0.0, 0.0, 0.0, 0.0, False
        return (_sf(s["pos"][i1]), _sf(s["vel"][i1]),
                _sf(s["pos"][i2]), _sf(s["vel"][i2]), True)
    except: return 0.0, 0.0, 0.0, 0.0, False


# ============================================================
# OmniGraph
# ============================================================
def setup(db: og.Database):
    st = _state(db); side = _side(db); c = _cfg(side)
    st["side"] = side
    st["state"] = ST_INIT
    st["prev"] = ST_INIT
    st["frame"] = 0; st["ccnt"] = 0
    st["target"] = 0.02
    st["hold_pos"] = 0.02
    st["pp1"] = 0.0; st["pp2"] = 0.0
    st["dc"] = None; st["art"] = 0; st["dof"] = {}; st["dc_retry"] = -9999
    _log(st, c, f"=== v4.2 POS {side} === node={st['path']} "
         f"MIN={c['MIN_POS']*1000:.1f}mm MAX={c['MAX_POS']*1000:.1f}mm "
         f"CLOSE_STEP={c['CLOSE_STEP']*1000:.2f} OPEN_STEP={c['OPEN_STEP']*1000:.2f} "
         f"INIT_CLOSE={c['INIT_CLOSE']}", True)
    _init_dc(st, c)


def cleanup(db: og.Database):
    st = _state(db); c = _cfg(st["side"])
    _log(st, c, "=== cleanup ===", True)


def compute(db: og.Database):
    st = _state(db); side = _side(db); c = _cfg(side)
    st["side"] = side
    st["frame"] += 1; st["prev"] = st["state"]

    # --- ROS2 ---
    if st["state"] != ST_INIT:
        arr = db.inputs.input_double_array
        if arr is not None:
            data = list(arr)
            if len(data) > 0:
                cmd = _sf(data[0], None)
                if cmd is not None:
                    if cmd > 0:
                        st["state"] = ST_OPEN; st["ccnt"] = 0
                        _log(st, c, f"→ OPEN", True)
                    elif cmd <= 0:
                        if st["state"] not in (ST_CLOSE, ST_HOLD):
                            st["state"] = ST_CLOSE; st["ccnt"] = 0
                            _log(st, c, f"→ CLOSE", True)

    # --- DC ---
    _init_dc(st, c)
    p1, v1, p2, v2, ok = _read2(st, c)
    gap = _gap(c, p1, p2) if ok else -1.0
    dp1 = abs(p1 - st["pp1"]) if ok else 999.0
    dp2 = abs(p2 - st["pp2"]) if ok else 999.0
    if ok: st["pp1"] = p1; st["pp2"] = p2

    # 接触检测函数 (闭合时用)
    def _check_contact():
        vel_ok = ok and abs(v1) < c["VEL_THRESH"] and abs(v2) < c["VEL_THRESH"]
        pos_ok = dp1 < c["POS_STABLE"] and dp2 < c["POS_STABLE"]
        not_limit = ok and p1 > c["USD_LOWER"] + 0.002 and p2 > c["USD_LOWER"] + 0.002
        return vel_ok and pos_ok and not_limit

    # ========================
    # INIT
    # ========================
    if st["state"] == ST_INIT:
        if st["frame"] <= c["WARMUP_FRAMES"]:
            # 用当前位置当 target, 防跳
            if ok:
                st["target"] = max((p1 + p2) / 2.0, c["MIN_POS"])
            else:
                st["target"] = 0.02
        else:
            if c["INIT_CLOSE"]:
                # 初始含托盘 → 闭合夹持
                st["target"] = max(st["target"] - c["CLOSE_STEP"], c["MIN_POS"])
                if _check_contact():
                    st["ccnt"] += 1
                else:
                    st["ccnt"] = 0
                if st["ccnt"] >= c["CONTACT_CONFIRM"]:
                    avg = (p1 + p2) / 2.0 if ok else st["target"]
                    st["hold_pos"] = max(avg - c["HOLD_PRELOAD"], c["MIN_POS"])
                    st["target"] = st["hold_pos"]
                    st["state"] = ST_HOLD
                    _log(st, c, f"INIT→HOLD contact gap={gap*1000:.1f}mm "
                         f"hold={st['hold_pos']*1000:.2f}mm", True)
                elif st["target"] <= c["MIN_POS"]:
                    st["hold_pos"] = c["MIN_POS"]
                    st["target"] = c["MIN_POS"]
                    st["state"] = ST_HOLD
                    _log(st, c, f"INIT→HOLD min_pos (no tray)", True)
            else:
                st["state"] = ST_IDLE
                st["target"] = c["MIN_POS"]
        _out(db, c, st["target"])
        if st["frame"] % 10 == 0:
            _log(st, c, f"INIT tgt={st['target']*1000:.2f} "
                 f"j1={p1*1000:.2f} j2={p2*1000:.2f} "
                 f"gap={'%.1f'%(gap*1000) if ok else '?'}mm cc={st['ccnt']}", True)
        return True

    # ========================
    # OPEN: target 增大到 MAX_POS
    # ========================
    if st["state"] == ST_OPEN:
        st["target"] = min(st["target"] + c["OPEN_STEP"], c["MAX_POS"])

    # ========================
    # CLOSE: target 减小, 力位检测
    # ========================
    elif st["state"] == ST_CLOSE:
        st["target"] = max(st["target"] - c["CLOSE_STEP"], c["MIN_POS"])
        if _check_contact():
            st["ccnt"] += 1
        else:
            st["ccnt"] = 0
        if st["ccnt"] >= c["CONTACT_CONFIRM"]:
            avg = (p1 + p2) / 2.0 if ok else st["target"]
            st["hold_pos"] = max(avg - c["HOLD_PRELOAD"], c["MIN_POS"])
            st["target"] = st["hold_pos"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD contact gap={gap*1000:.1f}mm "
                 f"hold={st['hold_pos']*1000:.2f}mm", True)
        elif st["target"] <= c["MIN_POS"]:
            st["hold_pos"] = c["MIN_POS"]
            st["target"] = c["MIN_POS"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD min_pos (no tray)", True)

    # ========================
    # HOLD: 锁定位置 (Drive 自动保持)
    # ========================
    elif st["state"] == ST_HOLD:
        st["target"] = st["hold_pos"]
        if ok and max(abs(v1), abs(v2)) > c["SLIP_VEL"]:
            st["state"] = ST_CLOSE; st["ccnt"] = 0
            st["target"] = max((p1 + p2) / 2.0, c["MIN_POS"])
            _log(st, c, f"→ re-CLOSE slip v1={v1:.4f} v2={v2:.4f}", True)

    # ========================
    # IDLE
    # ========================
    else:
        pass  # target 不变

    _out(db, c, st["target"])

    g = f"{gap*1000:.1f}" if gap > 0 else "?"
    dj = f"{(p1-p2)*1000:.2f}" if ok else "?"
    _log(st, c, f"{ST_NAME.get(st['state'],'?')} "
         f"tgt={st['target']*1000:.2f}mm gap={g}mm dj={dj}mm "
         f"j1={p1*1000:.2f} j2={p2*1000:.2f} "
         f"v1={v1:.5f} v2={v2:.5f} cc={st['ccnt']}")
    return True


def _out(db, c, target):
    """两个 joint 发相同 target → Drive 各自跟踪 → 等行程同步"""
    db.outputs.joint_names = [c["joint1"], c["joint2"]]
    db.outputs.position_cmds = [float(target), float(target)]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
