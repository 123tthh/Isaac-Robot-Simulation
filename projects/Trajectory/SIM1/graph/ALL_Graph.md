# Isaac Sim Action Graph 全量汇总报告
> SIM1 工程总纲: [`../PROJECT_OVERVIEW.md`](../PROJECT_OVERVIEW.md)
> 相关操作指导: [`../GAME_GUIDE.md`](../GAME_GUIDE.md)
> **生成时间:** 2026-05-28 11:29:14
> **根路径:** `/World/ActionGraphs`
> **当前更新:** 2026-05-28，夹爪控制改为 PGIA v4.2 位置控制脚本 `/home/gtk/Trajectory/SIM1/script_gripper_v4.2.py`。

> 注意：下方 “Script Node 源代码快照” 保留了导出时的 v4.1 历史快照；当前应以 v4.2 脚本和 `position_cmds -> positionCommand` 连接为准。

---

## 📊 概览: 共发现 4 个 Action Graph

## 📂 Graph: `Arm_Control_Graph`
路径: `/World/ActionGraphs/Arm_Control_Graph`

### 1. 节点与参数配置
| 节点名称 | 类型 | 核心参数配置 |
| :--- | :--- | :--- |
| `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | - |
| `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | - |
| `ros2_subscribe_joint_state` | `isaacsim.ros2.bridge.ROS2SubscribeJointState` | **topicName**: `arm_joint_cmd` |
| `articulation_controller` | `isaacsim.core.nodes.IsaacArticulationController` | **robotPath**: `/World/Robot`<br>**targetPrim**: `/World/Robot` |

### 2. 拓扑连线 (Connections)
| 数据源 (Source) | ➔ | 目标端 (Target) |
| :--- | :---: | :--- |
| `on_playback_tick` (outputs:tick) | ➔ | `ros2_subscribe_joint_state` (inputs:execIn) |
| `ros2_context` (outputs:context) | ➔ | `ros2_subscribe_joint_state` (inputs:context) |
| `ros2_subscribe_joint_state` (outputs:effortCommand) | ➔ | `articulation_controller` (inputs:effortCommand) |
| `ros2_subscribe_joint_state` (outputs:execOut) | ➔ | `articulation_controller` (inputs:execIn) |
| `ros2_subscribe_joint_state` (outputs:jointNames) | ➔ | `articulation_controller` (inputs:jointNames) |
| `ros2_subscribe_joint_state` (outputs:positionCommand) | ➔ | `articulation_controller` (inputs:positionCommand) |
| `ros2_subscribe_joint_state` (outputs:velocityCommand) | ➔ | `articulation_controller` (inputs:velocityCommand) |

------------------------------

## 📂 Graph: `State_Telemetry_Graph`
路径: `/World/ActionGraphs/State_Telemetry_Graph`

### 1. 节点与参数配置
| 节点名称 | 类型 | 核心参数配置 |
| :--- | :--- | :--- |
| `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | - |
| `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | - |
| `read_sim_time` | `isaacsim.core.nodes.IsaacReadSimulationTime` | - |
| `ros2_publish_clock` | `isaacsim.ros2.bridge.ROS2PublishClock` | - |
| `ros2_publish_joint_state` | `isaacsim.ros2.bridge.ROS2PublishJointState` | **topicName**: `isaac_joint_states`<br>**targetPrim**: `/World/Robot/base_link` |

### 2. 拓扑连线 (Connections)
| 数据源 (Source) | ➔ | 目标端 (Target) |
| :--- | :---: | :--- |
| `on_playback_tick` (outputs:tick) | ➔ | `ros2_publish_clock` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `ros2_publish_joint_state` (inputs:execIn) |
| `read_sim_time` (outputs:simulationTime) | ➔ | `ros2_publish_clock` (inputs:timeStamp) |
| `read_sim_time` (outputs:simulationTime) | ➔ | `ros2_publish_joint_state` (inputs:timeStamp) |
| `ros2_context` (outputs:context) | ➔ | `ros2_publish_clock` (inputs:context) |
| `ros2_context` (outputs:context) | ➔ | `ros2_publish_joint_state` (inputs:context) |

------------------------------

## 📂 Graph: `Camera_Publish_Graph`
路径: `/World/ActionGraphs/Camera_Publish_Graph`

### 1. 节点与参数配置
| 节点名称 | 类型 | 核心参数配置 |
| :--- | :--- | :--- |
| `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | - |
| `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | - |
| `cam_head_rgb` | `isaacsim.ros2.bridge.ROS2CameraHelper` | **topicName**: `/head_cam/color/image_raw` |
| `cam_head_depth` | `isaacsim.ros2.bridge.ROS2CameraHelper` | **topicName**: `/head_cam/depth/image_rect_raw/head_cam/depth/image_rect_raw` |
| `cam_left_rgb` | `isaacsim.ros2.bridge.ROS2CameraHelper` | **topicName**: `/left_cam/color/image_raw/left_cam/color/image_raw` |
| `cam_left_depth` | `isaacsim.ros2.bridge.ROS2CameraHelper` | **topicName**: `/left_cam/depth/image_rect_raw` |
| `cam_right_rgb` | `isaacsim.ros2.bridge.ROS2CameraHelper` | **topicName**: `/right_cam/color/image_raw` |
| `cam_right_depth` | `isaacsim.ros2.bridge.ROS2CameraHelper` | **topicName**: `/right_cam/depth/image_rect_raw` |

### 2. 拓扑连线 (Connections)
| 数据源 (Source) | ➔ | 目标端 (Target) |
| :--- | :---: | :--- |
| `on_playback_tick` (outputs:tick) | ➔ | `cam_head_depth` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `cam_head_rgb` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `cam_left_depth` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `cam_left_rgb` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `cam_right_depth` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `cam_right_rgb` (inputs:execIn) |
| `ros2_context` (outputs:context) | ➔ | `cam_head_depth` (inputs:context) |
| `ros2_context` (outputs:context) | ➔ | `cam_head_rgb` (inputs:context) |
| `ros2_context` (outputs:context) | ➔ | `cam_left_depth` (inputs:context) |
| `ros2_context` (outputs:context) | ➔ | `cam_left_rgb` (inputs:context) |
| `ros2_context` (outputs:context) | ➔ | `cam_right_depth` (inputs:context) |
| `ros2_context` (outputs:context) | ➔ | `cam_right_rgb` (inputs:context) |

------------------------------

## 📂 Graph: `Gripper_Control_Graph`
路径: `/World/ActionGraphs/Gripper_Control_Graph`

### 1. 节点与参数配置
| 节点名称 | 类型 | 核心参数配置 |
| :--- | :--- | :--- |
| `on_playback_tick` | `omni.graph.action.OnPlaybackTick` | - |
| `ros2_context` | `isaacsim.ros2.bridge.ROS2Context` | - |
| `sub_left_gripper` | `isaacsim.ros2.bridge.ROS2Subscriber` | **topicName**: `left_gripper_controller/commands`<br>**messageName**: `Float64MultiArray`<br>**messagePackage**: `std_msgs` |
| `sub_right_gripper` | `isaacsim.ros2.bridge.ROS2Subscriber` | **topicName**: `right_gripper_controller/commands`<br>**messageName**: `Float64MultiArray`<br>**messagePackage**: `std_msgs` |
| `script_right_gripper` | `omni.graph.scriptnode.ScriptNode` | - |
| `script_left_gripper` | `omni.graph.scriptnode.ScriptNode` | - |
| `artic_left_gripper` | `isaacsim.core.nodes.IsaacArticulationController` | **robotPath**: `/World/Robot`<br>**targetPrim**: `/World/Robot` |
| `artic_right_gripper` | `isaacsim.core.nodes.IsaacArticulationController` | **robotPath**: `/World/Robot`<br>**targetPrim**: `/World/Robot` |

### 2. 拓扑连线 (Connections)
| 数据源 (Source) | ➔ | 目标端 (Target) |
| :--- | :---: | :--- |
| `on_playback_tick` (outputs:tick) | ➔ | `script_left_gripper` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `script_right_gripper` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `sub_left_gripper` (inputs:execIn) |
| `on_playback_tick` (outputs:tick) | ➔ | `sub_right_gripper` (inputs:execIn) |
| `ros2_context` (outputs:context) | ➔ | `sub_left_gripper` (inputs:context) |
| `ros2_context` (outputs:context) | ➔ | `sub_right_gripper` (inputs:context) |
| `script_left_gripper` (outputs:position_cmds) | ➔ | `artic_left_gripper` (inputs:positionCommand) |
| `script_left_gripper` (outputs:execOut) | ➔ | `artic_left_gripper` (inputs:execIn) |
| `script_left_gripper` (outputs:joint_names) | ➔ | `artic_left_gripper` (inputs:jointNames) |
| `script_right_gripper` (outputs:position_cmds) | ➔ | `artic_right_gripper` (inputs:positionCommand) |
| `script_right_gripper` (outputs:execOut) | ➔ | `artic_right_gripper` (inputs:execIn) |
| `script_right_gripper` (outputs:joint_names) | ➔ | `artic_right_gripper` (inputs:jointNames) |
| `sub_left_gripper` (outputs:data) | ➔ | `script_left_gripper` (inputs:input_double_array) |
| `sub_right_gripper` (outputs:data) | ➔ | `script_right_gripper` (inputs:input_double_array) |

### 3. Script Node 源代码快照
#### 节点: `script_right_gripper`
```python
"""
PGIA 夹爪 Effort-PD 混合控制 v4.1 — 统一版 (左右共用)
================================================================
基于 v3.8 架构 (effort + builtins 隔离), 加入软件 PD 位控约束.

控制策略:
  每帧计算: effort = Kp * (target - current) - Kd * vel + gravity_comp
  不同状态设置不同的 target_pos, PD 自动跟踪

状态机:
  INIT  → target=托盘夹持位置, PD 闭合到接触 (初始含托盘)
  IDLE  → target=MAX_TRAVEL, PD 保持在张开位置 (无托盘)
  OPEN  → target=MAX_TRAVEL, PD 张开
  CLOSE → target 逐帧递减, PD 闭合, 接触检测
  HOLD  → target=锁定位置, PD 保持不漂移

5 项需求:
  1. 闭合最小行程 (含 gap) = 0.04m
     → MIN_GAP = 0.04, 即 j1=j2 ≈ (0.04-0.0197)/2 ≈ 0.0102
  2. MAX_TRAVEL = 0.24m (注: 超过 USD_UPPER=0.12, 会被钳位)
  3. HOLD 锁定接触位置不漂移 → PD 跟踪锁定位置
  4. 有托盘: 力控(PD 闭合+接触检测); 无托盘: 位控(PD 到 MAX)
  5. 初始含托盘 → INIT 直接进 CLOSE 逻辑, PD 闭合到接触

USD Drive 要求:
  stiffness = 0 (或很小, ≤50)
  damping = 0 (或很小, ≤20)
  maxForce = 500
  → 主要控制力来自 ScriptNode 的 PD effort

Graph 连线: effort_cmds → effortCommand (不是 positionCommand)
================================================================
"""

import os
import math
import builtins
from datetime import datetime

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
# 独立状态存储
# ============================================================
if not hasattr(builtins, "_PGIA_V41_STATE"):
    builtins._PGIA_V41_STATE = {}


# ============================================================
# 配置
# ============================================================
def _cfg(side):
    c = {
        "side": side,
        "joint1": f"{side}_PGIA_joint1",
        "joint2": f"{side}_PGIA_joint2",

        # ===== 几何参数 =====
        "BASE_GAP": 0.0197,          # j1=0,j2=0 时基准间距 (m)
        "USD_LOWER": 0.004,          # USD lowerLimit (m)
        "USD_UPPER": 0.12,           # USD upperLimit (m)

        # 需求2: 最大行程
        # 注: 0.24 > USD_UPPER(0.12), 实际被 USD 钳位到 0.12
        # 如果 USD upperLimit 已改为 0.24, 则生效
        "MAX_TRAVEL": 0.24,

        # 需求1: 闭合最小行程 (含 gap)
        # MIN_GAP = 0.04m → 单侧最小 pos = (0.04-0.0197)/2 ≈ 0.0102
        "MIN_GAP": 0.040,
        "MIN_POS": 0.010,            # (MIN_GAP - BASE_GAP) / 2, 取整

        # ===== PD 参数 =====
        # effort = Kp * (target - current) - Kd * velocity + gravity_comp
        #
        # 调参指南:
        #   Kp 越大跟踪越快但容易振荡
        #   Kd 越大阻尼越强, 抑制振荡但响应变慢
        #   稳定条件 (粗略): Kd > 2*sqrt(Kp*m), m≈0.3kg
        #     Kp=600, m=0.3 → Kd > 2*sqrt(180) ≈ 27
        "KP": 600.0,                 # 位置增益 (N/m)
        "KD": 40.0,                  # 速度增益 (N·s/m)
        # 调参:
        #   Kp=300, Kd=25: 慢但稳
        #   Kp=600, Kd=40: 推荐
        #   Kp=1200, Kd=80: 快但可能振荡

        # ===== 重力补偿 =====
        # joint 轴几乎竖直, link 重力沿轴分量:
        #   link1: 0.338kg × 9.81 × 0.999 ≈ 3.3N
        #   link2: 0.260kg × 9.81 × 0.999 ≈ 2.5N
        # 闭合方向 = effort < 0
        # 上夹爪 j1: 重力让它自然闭合(下落), 补偿方向=张开(正)
        # 下夹爪 j2: 重力让它自然张开(下坠), 补偿方向=闭合(负)
        "J1_GRAV_COMP": 3.5,         # j1 重力补偿 (正=张开, 抵消自然下落)
        "J2_GRAV_COMP": -3.0,        # j2 重力补偿 (负=闭合, 抵消自然下坠)
        # 调参: 先设为 0, 看哪个方向偏 → 加补偿

        # ===== 力控闭合 =====
        "CLOSE_STEP": 0.0003,        # 每帧 target 减小量 (m/frame)
        # 闭合速度 = CLOSE_STEP × 60 ≈ 18mm/s

        # ===== 接触检测 =====
        # PD 闭合中, 碰到托盘后:
        #   current 停住, target 继续减小
        #   error 增大 → effort 增大
        #   速度趋零
        # 检测: |effort| > threshold AND |vel| < vel_thresh
        "EFFORT_THRESHOLD": 15.0,    # PD 输出 effort 超此值 = 接触 (N)
        "VEL_THRESH": 0.002,         # 速度阈值 (m/s)
        "CONTACT_CONFIRM": 10,       # 连续确认帧数

        # ===== 保持 (HOLD) =====
        # 接触后锁定位置, PD 持续跟踪
        # 预紧: target 比实际位置再闭合一点, 产生持续夹持力
        "HOLD_PRELOAD": 0.002,       # 预紧量 (m), target = contact_pos - PRELOAD

        # ===== 脱落检测 =====
        "SLIP_VEL": 0.015,           # HOLD 中速度超此 = 脱落

        # ===== 预热 =====
        "WARMUP_FRAMES": 8,

        # ===== Effort 限幅 =====
        "EFFORT_MAX": 60.0,          # 最大 effort 绝对值 (N)

        # ===== EMA 滤波 =====
        "EMA_ALPHA": 0.2,            # 速度滤波系数

        # ===== 日志 =====
        "LOG_DIR": "/home/gtk/ros2_log",
        "LOG_INTERVAL": 30,
    }

    # 右侧托盘可能更重, j2 补偿可能需要更大
    if side == "right":
        c["J2_GRAV_COMP"] = -5.0

    c["LOG_FILE"] = f'{c["LOG_DIR"]}/{side}_gripper_v41.log'
    return c


# ============================================================
# 节点身份识别 (继承 v3.8)
# ============================================================
def _node_path(db):
    for attr_name in ["abi_node", "node"]:
        try:
            n = getattr(db, attr_name)
            for method_name in ["get_prim_path", "get_path"]:
                try:
                    p = str(getattr(n, method_name)())
                    if p and "script_" in p: return p
                except: pass
        except: pass
    return "unknown"

def _side_from_db(db):
    p = _node_path(db).lower()
    if "right" in p: return "right"
    return "left"

def _state_for(db):
    path = _node_path(db)
    side = _side_from_db(db)
    key = f"{path}:{side}"
    store = builtins._PGIA_V41_STATE
    if key not in store:
        store[key] = {
            "path": path, "side": side,
            "state": ST_INIT, "prev": ST_INIT,
            "frame": 0, "ccnt": 0,
            "target": 0.0,           # PD 目标位置
            "hold_pos": 0.0,         # HOLD 锁定位置
            "fv1": 0.0, "fv2": 0.0,  # EMA 滤波速度
            "pp1": 0.0, "pp2": 0.0,  # 上一帧位置
            "pd_effort1": 0.0,       # 上一帧 PD 输出 (用于接触检测)
            "pd_effort2": 0.0,
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
    carb.log_info(f"[{c['side']}_v41] {msg}")


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
# ★★★ 核心: 软件 PD 控制器 ★★★
# ============================================================
def _pd_effort(c, st, target, p1, v1_raw, p2, v2_raw, ok):
    """
    对 j1 j2 分别计算:
      effort = Kp * (target - current) - Kd * filtered_vel + gravity_comp

    返回 (ej1, ej2), 已限幅
    """
    if not ok:
        return 0.0, 0.0

    # EMA 滤波速度
    alpha = c["EMA_ALPHA"]
    st["fv1"] = alpha * v1_raw + (1.0 - alpha) * st["fv1"]
    st["fv2"] = alpha * v2_raw + (1.0 - alpha) * st["fv2"]

    Kp = c["KP"]
    Kd = c["KD"]
    lim = c["EFFORT_MAX"]

    # j1 (上夹爪)
    err1 = target - p1
    ej1 = Kp * err1 - Kd * st["fv1"] + c["J1_GRAV_COMP"]
    ej1 = float(np.clip(ej1, -lim, lim))

    # j2 (下夹爪)
    err2 = target - p2
    ej2 = Kp * err2 - Kd * st["fv2"] + c["J2_GRAV_COMP"]
    ej2 = float(np.clip(ej2, -lim, lim))

    # 保存供接触检测用
    st["pd_effort1"] = ej1
    st["pd_effort2"] = ej2

    return ej1, ej2


# ============================================================
# OmniGraph
# ============================================================
def setup(db: og.Database):
    st = _state_for(db)
    side = _side_from_db(db)
    c = _cfg(side)

    st["side"] = side
    st["state"] = ST_INIT
    st["prev"] = ST_INIT
    st["frame"] = 0; st["ccnt"] = 0
    st["target"] = c["MIN_POS"]      # 初始含托盘 → 从小行程开始
    st["hold_pos"] = c["MIN_POS"]
    st["fv1"] = 0.0; st["fv2"] = 0.0
    st["pp1"] = 0.0; st["pp2"] = 0.0
    st["pd_effort1"] = 0.0; st["pd_effort2"] = 0.0
    st["dc"] = None; st["art"] = 0; st["dof"] = {}; st["dc_retry"] = -9999

    max_gap = c["BASE_GAP"] + 2 * min(c["MAX_TRAVEL"], c["USD_UPPER"])
    _log(st, c, f"=== v4.1 {side} === "
         f"Kp={c['KP']:.0f} Kd={c['KD']:.0f} "
         f"J1gc={c['J1_GRAV_COMP']:.1f} J2gc={c['J2_GRAV_COMP']:.1f} "
         f"MIN_GAP={c['MIN_GAP']*1000:.0f}mm MAX_GAP={max_gap*1000:.1f}mm "
         f"ETH={c['EFFORT_THRESHOLD']:.0f}N STEP={c['CLOSE_STEP']*1000:.2f}mm/f", True)
    _init_dc(st, c)


def cleanup(db: og.Database):
    st = _state_for(db)
    c = _cfg(st["side"])
    _log(st, c, "=== cleanup ===", True)


def compute(db: og.Database):
    st = _state_for(db)
    side = _side_from_db(db)
    c = _cfg(side)
    st["side"] = side
    st["frame"] += 1; st["prev"] = st["state"]

    # 实际最大行程 (被 USD 限制)
    actual_max = min(c["MAX_TRAVEL"], c["USD_UPPER"])

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
                        st["target"] = actual_max
                        _log(st, c, f"→ OPEN target={actual_max*1000:.1f}mm", True)
                    elif cmd <= 0:
                        if st["state"] not in (ST_CLOSE, ST_HOLD):
                            st["state"] = ST_CLOSE; st["ccnt"] = 0
                            # 从当前位置开始闭合
                            _log(st, c, f"→ CLOSE from={st['target']*1000:.1f}mm", True)

    # --- DC ---
    _init_dc(st, c)
    p1, v1, p2, v2, ok = _read2(st, c)
    gap = _gap(c, p1, p2) if ok else -1.0

    # ========================
    # INIT: 初始含托盘, 直接闭合夹住
    # ========================
    if st["state"] == ST_INIT:
        if st["frame"] <= c["WARMUP_FRAMES"]:
            # 预热: 用当前位置作为 target, 防止跳动
            if ok:
                avg = (p1 + p2) / 2.0
                st["target"] = max(avg, c["MIN_POS"])
            else:
                st["target"] = c["MIN_POS"]
            ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)
            _log(st, c, f"warmup f={st['frame']} tgt={st['target']*1000:.1f}", True)
        else:
            # 逐帧闭合 target, PD 跟踪
            st["target"] = max(st["target"] - c["CLOSE_STEP"], c["MIN_POS"])

            ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

            # 接触检测
            max_eff = max(abs(st["pd_effort1"]), abs(st["pd_effort2"]))
            max_vel = max(abs(v1), abs(v2)) if ok else 999.0
            contacted = max_eff >= c["EFFORT_THRESHOLD"] and max_vel < c["VEL_THRESH"]

            if contacted: st["ccnt"] += 1
            else: st["ccnt"] = 0

            if st["ccnt"] >= c["CONTACT_CONFIRM"]:
                avg = (p1 + p2) / 2.0 if ok else st["target"]
                st["hold_pos"] = avg - c["HOLD_PRELOAD"]
                st["target"] = st["hold_pos"]
                st["state"] = ST_HOLD
                _log(st, c, f"INIT→HOLD contact "
                     f"gap={gap*1000:.1f}mm hold={st['hold_pos']*1000:.2f}mm "
                     f"eff={max_eff:.1f}N", True)
            elif st["target"] <= c["MIN_POS"]:
                # 到最小行程, 没检测到接触 (可能空夹)
                st["hold_pos"] = c["MIN_POS"]
                st["target"] = c["MIN_POS"]
                st["state"] = ST_HOLD
                _log(st, c, f"INIT→HOLD min_pos (no contact)", True)

        _out(db, c, ej1, ej2)
        if st["frame"] % 10 == 0:
            _log(st, c, f"INIT tgt={st['target']*1000:.2f} "
                 f"j1={p1*1000:.2f} j2={p2*1000:.2f} "
                 f"eff1={st['pd_effort1']:.1f} eff2={st['pd_effort2']:.1f} "
                 f"gap={'%.1f'%(gap*1000) if ok else '?'}mm cc={st['ccnt']}", True)
        return True

    # ========================
    # OPEN: PD 到 MAX_TRAVEL
    # ========================
    if st["state"] == ST_OPEN:
        st["target"] = actual_max
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

    # ========================
    # CLOSE: PD 闭合 + 接触检测
    # ========================
    elif st["state"] == ST_CLOSE:
        st["target"] = max(st["target"] - c["CLOSE_STEP"], c["MIN_POS"])
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

        # 接触检测: PD effort 增大 + 速度趋零
        max_eff = max(abs(st["pd_effort1"]), abs(st["pd_effort2"]))
        max_vel = max(abs(v1), abs(v2)) if ok else 999.0
        not_limit = ok and p1 > c["USD_LOWER"] + 0.002 and p2 > c["USD_LOWER"] + 0.002
        contacted = max_eff >= c["EFFORT_THRESHOLD"] and max_vel < c["VEL_THRESH"] and not_limit

        if contacted: st["ccnt"] += 1
        else: st["ccnt"] = 0

        if st["ccnt"] >= c["CONTACT_CONFIRM"]:
            avg = (p1 + p2) / 2.0 if ok else st["target"]
            st["hold_pos"] = avg - c["HOLD_PRELOAD"]
            st["target"] = st["hold_pos"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD contact gap={gap*1000:.1f}mm "
                 f"hold={st['hold_pos']*1000:.2f}mm eff={max_eff:.1f}N", True)
        elif gap <= c["MIN_GAP"] and ok:
            st["hold_pos"] = c["MIN_POS"]
            st["target"] = c["MIN_POS"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD min_gap={gap*1000:.1f}mm", True)
        elif st["target"] <= c["MIN_POS"]:
            st["hold_pos"] = c["MIN_POS"]
            st["target"] = c["MIN_POS"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD min_pos", True)

    # ========================
    # HOLD: PD 锁定位置 (需求3: 不漂移)
    # ========================
    elif st["state"] == ST_HOLD:
        st["target"] = st["hold_pos"]
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

        # 脱落检测
        if ok and max(abs(v1), abs(v2)) > c["SLIP_VEL"]:
            st["state"] = ST_CLOSE; st["ccnt"] = 0
            # 从当前位置重新开始闭合
            st["target"] = max((p1 + p2) / 2.0, c["MIN_POS"])
            _log(st, c, f"→ re-CLOSE slip v1={v1:.4f} v2={v2:.4f}", True)

    # ========================
    # IDLE: PD 保持在 MAX_TRAVEL (需求4: 无托盘)
    # ========================
    else:
        st["target"] = actual_max
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

    # --- 输出 ---
    _out(db, c, ej1, ej2)

    g = f"{gap*1000:.1f}" if gap > 0 else "?"
    dj = f"{(p1-p2)*1000:.2f}" if ok else "?"
    _log(st, c, f"{ST_NAME.get(st['state'],'?')} "
         f"tgt={st['target']*1000:.2f} "
         f"ej1={ej1:.1f} ej2={ej2:.1f} "
         f"gap={g}mm dj={dj}mm "
         f"j1={p1*1000:.2f} j2={p2*1000:.2f} "
         f"v1={v1:.5f} v2={v2:.5f} "
         f"pe1={st['pd_effort1']:.1f} pe2={st['pd_effort2']:.1f} "
         f"cc={st['ccnt']}")
    return True


def _out(db, c, ej1, ej2):
    db.outputs.joint_names = [c["joint1"], c["joint2"]]
    db.outputs.effort_cmds = [float(ej1), float(ej2)]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
```
#### 节点: `script_left_gripper`
```python
"""
PGIA 夹爪 Effort-PD 混合控制 v4.1 — 统一版 (左右共用)
================================================================
基于 v3.8 架构 (effort + builtins 隔离), 加入软件 PD 位控约束.

控制策略:
  每帧计算: effort = Kp * (target - current) - Kd * vel + gravity_comp
  不同状态设置不同的 target_pos, PD 自动跟踪

状态机:
  INIT  → target=托盘夹持位置, PD 闭合到接触 (初始含托盘)
  IDLE  → target=MAX_TRAVEL, PD 保持在张开位置 (无托盘)
  OPEN  → target=MAX_TRAVEL, PD 张开
  CLOSE → target 逐帧递减, PD 闭合, 接触检测
  HOLD  → target=锁定位置, PD 保持不漂移

5 项需求:
  1. 闭合最小行程 (含 gap) = 0.04m
     → MIN_GAP = 0.04, 即 j1=j2 ≈ (0.04-0.0197)/2 ≈ 0.0102
  2. MAX_TRAVEL = 0.24m (注: 超过 USD_UPPER=0.12, 会被钳位)
  3. HOLD 锁定接触位置不漂移 → PD 跟踪锁定位置
  4. 有托盘: 力控(PD 闭合+接触检测); 无托盘: 位控(PD 到 MAX)
  5. 初始含托盘 → INIT 直接进 CLOSE 逻辑, PD 闭合到接触

USD Drive 要求:
  stiffness = 0 (或很小, ≤50)
  damping = 0 (或很小, ≤20)
  maxForce = 500
  → 主要控制力来自 ScriptNode 的 PD effort

Graph 连线: effort_cmds → effortCommand (不是 positionCommand)
================================================================
"""

import os
import math
import builtins
from datetime import datetime

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
# 独立状态存储
# ============================================================
if not hasattr(builtins, "_PGIA_V41_STATE"):
    builtins._PGIA_V41_STATE = {}


# ============================================================
# 配置
# ============================================================
def _cfg(side):
    c = {
        "side": side,
        "joint1": f"{side}_PGIA_joint1",
        "joint2": f"{side}_PGIA_joint2",

        # ===== 几何参数 =====
        "BASE_GAP": 0.0197,          # j1=0,j2=0 时基准间距 (m)
        "USD_LOWER": 0.004,          # USD lowerLimit (m)
        "USD_UPPER": 0.12,           # USD upperLimit (m)

        # 需求2: 最大行程
        # 注: 0.24 > USD_UPPER(0.12), 实际被 USD 钳位到 0.12
        # 如果 USD upperLimit 已改为 0.24, 则生效
        "MAX_TRAVEL": 0.24,

        # 需求1: 闭合最小行程 (含 gap)
        # MIN_GAP = 0.04m → 单侧最小 pos = (0.04-0.0197)/2 ≈ 0.0102
        "MIN_GAP": 0.040,
        "MIN_POS": 0.010,            # (MIN_GAP - BASE_GAP) / 2, 取整

        # ===== PD 参数 =====
        # effort = Kp * (target - current) - Kd * velocity + gravity_comp
        #
        # 调参指南:
        #   Kp 越大跟踪越快但容易振荡
        #   Kd 越大阻尼越强, 抑制振荡但响应变慢
        #   稳定条件 (粗略): Kd > 2*sqrt(Kp*m), m≈0.3kg
        #     Kp=600, m=0.3 → Kd > 2*sqrt(180) ≈ 27
        "KP": 600.0,                 # 位置增益 (N/m)
        "KD": 40.0,                  # 速度增益 (N·s/m)
        # 调参:
        #   Kp=300, Kd=25: 慢但稳
        #   Kp=600, Kd=40: 推荐
        #   Kp=1200, Kd=80: 快但可能振荡

        # ===== 重力补偿 =====
        # joint 轴几乎竖直, link 重力沿轴分量:
        #   link1: 0.338kg × 9.81 × 0.999 ≈ 3.3N
        #   link2: 0.260kg × 9.81 × 0.999 ≈ 2.5N
        # 闭合方向 = effort < 0
        # 上夹爪 j1: 重力让它自然闭合(下落), 补偿方向=张开(正)
        # 下夹爪 j2: 重力让它自然张开(下坠), 补偿方向=闭合(负)
        "J1_GRAV_COMP": 3.5,         # j1 重力补偿 (正=张开, 抵消自然下落)
        "J2_GRAV_COMP": -3.0,        # j2 重力补偿 (负=闭合, 抵消自然下坠)
        # 调参: 先设为 0, 看哪个方向偏 → 加补偿

        # ===== 力控闭合 =====
        "CLOSE_STEP": 0.0003,        # 每帧 target 减小量 (m/frame)
        # 闭合速度 = CLOSE_STEP × 60 ≈ 18mm/s

        # ===== 接触检测 =====
        # PD 闭合中, 碰到托盘后:
        #   current 停住, target 继续减小
        #   error 增大 → effort 增大
        #   速度趋零
        # 检测: |effort| > threshold AND |vel| < vel_thresh
        "EFFORT_THRESHOLD": 15.0,    # PD 输出 effort 超此值 = 接触 (N)
        "VEL_THRESH": 0.002,         # 速度阈值 (m/s)
        "CONTACT_CONFIRM": 10,       # 连续确认帧数

        # ===== 保持 (HOLD) =====
        # 接触后锁定位置, PD 持续跟踪
        # 预紧: target 比实际位置再闭合一点, 产生持续夹持力
        "HOLD_PRELOAD": 0.002,       # 预紧量 (m), target = contact_pos - PRELOAD

        # ===== 脱落检测 =====
        "SLIP_VEL": 0.015,           # HOLD 中速度超此 = 脱落

        # ===== 预热 =====
        "WARMUP_FRAMES": 8,

        # ===== Effort 限幅 =====
        "EFFORT_MAX": 60.0,          # 最大 effort 绝对值 (N)

        # ===== EMA 滤波 =====
        "EMA_ALPHA": 0.2,            # 速度滤波系数

        # ===== 日志 =====
        "LOG_DIR": "/home/gtk/ros2_log",
        "LOG_INTERVAL": 30,
    }

    # 右侧托盘可能更重, j2 补偿可能需要更大
    if side == "right":
        c["J2_GRAV_COMP"] = -5.0

    c["LOG_FILE"] = f'{c["LOG_DIR"]}/{side}_gripper_v41.log'
    return c


# ============================================================
# 节点身份识别 (继承 v3.8)
# ============================================================
def _node_path(db):
    for attr_name in ["abi_node", "node"]:
        try:
            n = getattr(db, attr_name)
            for method_name in ["get_prim_path", "get_path"]:
                try:
                    p = str(getattr(n, method_name)())
                    if p and "script_" in p: return p
                except: pass
        except: pass
    return "unknown"

def _side_from_db(db):
    p = _node_path(db).lower()
    if "right" in p: return "right"
    return "left"

def _state_for(db):
    path = _node_path(db)
    side = _side_from_db(db)
    key = f"{path}:{side}"
    store = builtins._PGIA_V41_STATE
    if key not in store:
        store[key] = {
            "path": path, "side": side,
            "state": ST_INIT, "prev": ST_INIT,
            "frame": 0, "ccnt": 0,
            "target": 0.0,           # PD 目标位置
            "hold_pos": 0.0,         # HOLD 锁定位置
            "fv1": 0.0, "fv2": 0.0,  # EMA 滤波速度
            "pp1": 0.0, "pp2": 0.0,  # 上一帧位置
            "pd_effort1": 0.0,       # 上一帧 PD 输出 (用于接触检测)
            "pd_effort2": 0.0,
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
    carb.log_info(f"[{c['side']}_v41] {msg}")


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
# ★★★ 核心: 软件 PD 控制器 ★★★
# ============================================================
def _pd_effort(c, st, target, p1, v1_raw, p2, v2_raw, ok):
    """
    对 j1 j2 分别计算:
      effort = Kp * (target - current) - Kd * filtered_vel + gravity_comp

    返回 (ej1, ej2), 已限幅
    """
    if not ok:
        return 0.0, 0.0

    # EMA 滤波速度
    alpha = c["EMA_ALPHA"]
    st["fv1"] = alpha * v1_raw + (1.0 - alpha) * st["fv1"]
    st["fv2"] = alpha * v2_raw + (1.0 - alpha) * st["fv2"]

    Kp = c["KP"]
    Kd = c["KD"]
    lim = c["EFFORT_MAX"]

    # j1 (上夹爪)
    err1 = target - p1
    ej1 = Kp * err1 - Kd * st["fv1"] + c["J1_GRAV_COMP"]
    ej1 = float(np.clip(ej1, -lim, lim))

    # j2 (下夹爪)
    err2 = target - p2
    ej2 = Kp * err2 - Kd * st["fv2"] + c["J2_GRAV_COMP"]
    ej2 = float(np.clip(ej2, -lim, lim))

    # 保存供接触检测用
    st["pd_effort1"] = ej1
    st["pd_effort2"] = ej2

    return ej1, ej2


# ============================================================
# OmniGraph
# ============================================================
def setup(db: og.Database):
    st = _state_for(db)
    side = _side_from_db(db)
    c = _cfg(side)

    st["side"] = side
    st["state"] = ST_INIT
    st["prev"] = ST_INIT
    st["frame"] = 0; st["ccnt"] = 0
    st["target"] = c["MIN_POS"]      # 初始含托盘 → 从小行程开始
    st["hold_pos"] = c["MIN_POS"]
    st["fv1"] = 0.0; st["fv2"] = 0.0
    st["pp1"] = 0.0; st["pp2"] = 0.0
    st["pd_effort1"] = 0.0; st["pd_effort2"] = 0.0
    st["dc"] = None; st["art"] = 0; st["dof"] = {}; st["dc_retry"] = -9999

    max_gap = c["BASE_GAP"] + 2 * min(c["MAX_TRAVEL"], c["USD_UPPER"])
    _log(st, c, f"=== v4.1 {side} === "
         f"Kp={c['KP']:.0f} Kd={c['KD']:.0f} "
         f"J1gc={c['J1_GRAV_COMP']:.1f} J2gc={c['J2_GRAV_COMP']:.1f} "
         f"MIN_GAP={c['MIN_GAP']*1000:.0f}mm MAX_GAP={max_gap*1000:.1f}mm "
         f"ETH={c['EFFORT_THRESHOLD']:.0f}N STEP={c['CLOSE_STEP']*1000:.2f}mm/f", True)
    _init_dc(st, c)


def cleanup(db: og.Database):
    st = _state_for(db)
    c = _cfg(st["side"])
    _log(st, c, "=== cleanup ===", True)


def compute(db: og.Database):
    st = _state_for(db)
    side = _side_from_db(db)
    c = _cfg(side)
    st["side"] = side
    st["frame"] += 1; st["prev"] = st["state"]

    # 实际最大行程 (被 USD 限制)
    actual_max = min(c["MAX_TRAVEL"], c["USD_UPPER"])

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
                        st["target"] = actual_max
                        _log(st, c, f"→ OPEN target={actual_max*1000:.1f}mm", True)
                    elif cmd <= 0:
                        if st["state"] not in (ST_CLOSE, ST_HOLD):
                            st["state"] = ST_CLOSE; st["ccnt"] = 0
                            # 从当前位置开始闭合
                            _log(st, c, f"→ CLOSE from={st['target']*1000:.1f}mm", True)

    # --- DC ---
    _init_dc(st, c)
    p1, v1, p2, v2, ok = _read2(st, c)
    gap = _gap(c, p1, p2) if ok else -1.0

    # ========================
    # INIT: 初始含托盘, 直接闭合夹住
    # ========================
    if st["state"] == ST_INIT:
        if st["frame"] <= c["WARMUP_FRAMES"]:
            # 预热: 用当前位置作为 target, 防止跳动
            if ok:
                avg = (p1 + p2) / 2.0
                st["target"] = max(avg, c["MIN_POS"])
            else:
                st["target"] = c["MIN_POS"]
            ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)
            _log(st, c, f"warmup f={st['frame']} tgt={st['target']*1000:.1f}", True)
        else:
            # 逐帧闭合 target, PD 跟踪
            st["target"] = max(st["target"] - c["CLOSE_STEP"], c["MIN_POS"])

            ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

            # 接触检测
            max_eff = max(abs(st["pd_effort1"]), abs(st["pd_effort2"]))
            max_vel = max(abs(v1), abs(v2)) if ok else 999.0
            contacted = max_eff >= c["EFFORT_THRESHOLD"] and max_vel < c["VEL_THRESH"]

            if contacted: st["ccnt"] += 1
            else: st["ccnt"] = 0

            if st["ccnt"] >= c["CONTACT_CONFIRM"]:
                avg = (p1 + p2) / 2.0 if ok else st["target"]
                st["hold_pos"] = avg - c["HOLD_PRELOAD"]
                st["target"] = st["hold_pos"]
                st["state"] = ST_HOLD
                _log(st, c, f"INIT→HOLD contact "
                     f"gap={gap*1000:.1f}mm hold={st['hold_pos']*1000:.2f}mm "
                     f"eff={max_eff:.1f}N", True)
            elif st["target"] <= c["MIN_POS"]:
                # 到最小行程, 没检测到接触 (可能空夹)
                st["hold_pos"] = c["MIN_POS"]
                st["target"] = c["MIN_POS"]
                st["state"] = ST_HOLD
                _log(st, c, f"INIT→HOLD min_pos (no contact)", True)

        _out(db, c, ej1, ej2)
        if st["frame"] % 10 == 0:
            _log(st, c, f"INIT tgt={st['target']*1000:.2f} "
                 f"j1={p1*1000:.2f} j2={p2*1000:.2f} "
                 f"eff1={st['pd_effort1']:.1f} eff2={st['pd_effort2']:.1f} "
                 f"gap={'%.1f'%(gap*1000) if ok else '?'}mm cc={st['ccnt']}", True)
        return True

    # ========================
    # OPEN: PD 到 MAX_TRAVEL
    # ========================
    if st["state"] == ST_OPEN:
        st["target"] = actual_max
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

    # ========================
    # CLOSE: PD 闭合 + 接触检测
    # ========================
    elif st["state"] == ST_CLOSE:
        st["target"] = max(st["target"] - c["CLOSE_STEP"], c["MIN_POS"])
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

        # 接触检测: PD effort 增大 + 速度趋零
        max_eff = max(abs(st["pd_effort1"]), abs(st["pd_effort2"]))
        max_vel = max(abs(v1), abs(v2)) if ok else 999.0
        not_limit = ok and p1 > c["USD_LOWER"] + 0.002 and p2 > c["USD_LOWER"] + 0.002
        contacted = max_eff >= c["EFFORT_THRESHOLD"] and max_vel < c["VEL_THRESH"] and not_limit

        if contacted: st["ccnt"] += 1
        else: st["ccnt"] = 0

        if st["ccnt"] >= c["CONTACT_CONFIRM"]:
            avg = (p1 + p2) / 2.0 if ok else st["target"]
            st["hold_pos"] = avg - c["HOLD_PRELOAD"]
            st["target"] = st["hold_pos"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD contact gap={gap*1000:.1f}mm "
                 f"hold={st['hold_pos']*1000:.2f}mm eff={max_eff:.1f}N", True)
        elif gap <= c["MIN_GAP"] and ok:
            st["hold_pos"] = c["MIN_POS"]
            st["target"] = c["MIN_POS"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD min_gap={gap*1000:.1f}mm", True)
        elif st["target"] <= c["MIN_POS"]:
            st["hold_pos"] = c["MIN_POS"]
            st["target"] = c["MIN_POS"]
            st["state"] = ST_HOLD
            _log(st, c, f"→ HOLD min_pos", True)

    # ========================
    # HOLD: PD 锁定位置 (需求3: 不漂移)
    # ========================
    elif st["state"] == ST_HOLD:
        st["target"] = st["hold_pos"]
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

        # 脱落检测
        if ok and max(abs(v1), abs(v2)) > c["SLIP_VEL"]:
            st["state"] = ST_CLOSE; st["ccnt"] = 0
            # 从当前位置重新开始闭合
            st["target"] = max((p1 + p2) / 2.0, c["MIN_POS"])
            _log(st, c, f"→ re-CLOSE slip v1={v1:.4f} v2={v2:.4f}", True)

    # ========================
    # IDLE: PD 保持在 MAX_TRAVEL (需求4: 无托盘)
    # ========================
    else:
        st["target"] = actual_max
        ej1, ej2 = _pd_effort(c, st, st["target"], p1, v1, p2, v2, ok)

    # --- 输出 ---
    _out(db, c, ej1, ej2)

    g = f"{gap*1000:.1f}" if gap > 0 else "?"
    dj = f"{(p1-p2)*1000:.2f}" if ok else "?"
    _log(st, c, f"{ST_NAME.get(st['state'],'?')} "
         f"tgt={st['target']*1000:.2f} "
         f"ej1={ej1:.1f} ej2={ej2:.1f} "
         f"gap={g}mm dj={dj}mm "
         f"j1={p1*1000:.2f} j2={p2*1000:.2f} "
         f"v1={v1:.5f} v2={v2:.5f} "
         f"pe1={st['pd_effort1']:.1f} pe2={st['pd_effort2']:.1f} "
         f"cc={st['ccnt']}")
    return True


def _out(db, c, ej1, ej2):
    db.outputs.joint_names = [c["joint1"], c["joint2"]]
    db.outputs.effort_cmds = [float(ej1), float(ej2)]
    db.outputs.execOut = og.ExecutionAttributeState.ENABLED
```

------------------------------
