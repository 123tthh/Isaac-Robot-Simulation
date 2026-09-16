> Historical upstream document / 上游历史文档。Current instructions: [中文](../getting-started.zh-CN.md) / [English](../getting-started.md).

# SIM1 工程纲要

本文是规范化工程中 `projects/teleoperation/sim1` 的模块总览。项目级启动方法以
`${PROJECT_ROOT}/docs/archive/startup-guide-alias.zh-CN.md` 为准。

## 1. 工程目标

SIM1 用于 Isaac Sim 5.1.0 + ROS 2 Humble 运行环境 + OCS2 的双臂夹爪示教、回放、诊断和数据转换；ROS 2 Rolling 文档用于 API 核对。

核心能力：

- 启动前检查 ROS/Isaac/OCS2 关键 topic、service 和脚本环境。
- 手动示教左右臂 OCS2 target，并同步记录夹爪状态、关节位置、关节力矩和诊断 CSV。
- 可选同步启动 ROS2 相机 recorder，记录 3 路 RGB+D 图像、帧 timestamp 和 summary。
- 支持真实差速底盘 `/sim1/base_diff_cmd` 的独立轨迹运动、独立 CSV 记录、与 teach 联合示教和回放。
- 支持终端单键示教和 pygame 多键连续速度示教。
- 回放原始或清洗后的 OCS2 target 轨迹。
- 清洗人工示教轨迹，删除“时间流动但无动作”的观察段。
- 将清洗后的轨迹和相机数据转换为本地 LeRobotDataset v3.0 与 v2.1 结构。
- 为 Isaac Sim Script Editor 提供只读诊断脚本，排查 joint1-7 drive、Action Graph、力矩消失等问题。

## 2. 最常用入口

```bash
cd ${PROJECT_ROOT}
./scripts/project_control.sh sim1-check
./scripts/project_control.sh sim1-reset
./scripts/project_control.sh sim1-teach
```

正式手动示教推荐使用 pygame 多键模式并同步采集 RGB+D：

```bash
./game_control.sh teach --input-mode pygame \
  --linear-speed 0.025 \
  --angular-speed-deg 4.0 \
  --control-rate 50 \
  --max-position-error 0.08 \
  --hard-position-error 0.15 \
  --max-orientation-error-deg 15.0 \
  --watchdog-position-error 0.15 \
  --watchdog-orientation-error-deg 30.0 \
  --watchdog-severe-position-error 0.30 \
  --watchdog-severe-orientation-error-deg 60.0 \
  --right-joint1-lower-safety -2.2 \
  --right-joint1-upper-safety 1.2 \
  --record-cameras \
  --camera-depth-save-every 1
```

这个命令会同时记录轨迹、诊断、夹爪、关节、3 路 RGB 视频和 3 路 depth npy。更多参数说明见 [GAME_GUIDE.md](GAME_GUIDE.md) 的“示教模式”小节。

## 3. 运行链路

```text
Isaac Sim scene
  -> ROS 2 topics
  -> OCS2 current pose / FSM / target topics
  -> keyboard_ocs2_gripper_teleop.py
  -> trace_data/raw/*.csv + *.diagnostics.csv + optional */camera/
  -> trace_cleaning/scripts
  -> trace_data/cleaned/*.csv + optional */camera/
  -> trace_data/lerobot_v30/ + optional trace_data/lerobot_v21/
```

主要控制 topic：

```text
/left_target/stamped
/right_target/stamped
/left_current_pose
/right_current_pose
/left_gripper_controller/commands
/right_gripper_controller/commands
/joint_states
/isaac_joint_states
/fsm_command
/sim1/base_diff_cmd
```

## 4. 目录地图

```text
SIM1/
  game_control.sh                         统一入口：check/reset/teach/demo/direct-demo
  keyboard_ocs2_gripper_teleop.py         示教器：发布 OCS2 target，记录 CSV 和诊断 CSV
  base_drive_path_runner.py               独立底盘轨迹运动脚本：前进/后退/圆弧 90°/path-demo
  base_drive_recorder.py                  独立底盘 CSV recorder，写入 trace_data/base_raw/
  tools/camera_recorder.py                独立 ROS2 相机 recorder，可由示教器 subprocess 启动
  replay_ocs2_target_trace.py             OCS2 target CSV 回放
  replay_lerobot_episode.py               LeRobot 关节轨迹直连回放
  TRACE_DATA_FORMAT.md                    轨迹、诊断、清洗、LeRobot 转换格式说明
  GAME_GUIDE.md                           操作指导书：启动、示教、回放、键位、常见问题
  PROJECT_OVERVIEW.md                     本文件
  trace_data/
    raw/                                  原始示教 CSV、诊断 CSV、失败标注
    base_raw/                             底盘独立记录 CSV
    cleaned/                              清洗后可回放 CSV、清洗报告、裁剪后相机数据
    direct/                               直连回放检查 CSV
    lerobot_v30/                          LeRobot dataset v3.0 本地输出
    lerobot_v21/                          LeRobot dataset v2.1 本地输出
  trace_cleaning/scripts/                 清洗和 LeRobot 转换脚本
  isaac_script_editor_diagnostics/        Isaac Script Editor 只读诊断脚本和输出
  graph/                                  Isaac Stage / Action Graph / 夹爪诊断导出
```

## 5. 说明文档地图

- [GAME_GUIDE.md](GAME_GUIDE.md)：面向操作，说明启动顺序、示教模式命令、键位、回放、常见问题。
- [TRACE_DATA_FORMAT.md](TRACE_DATA_FORMAT.md)：面向数据，说明 raw/diagnostics/cleaned/camera/label/LeRobot v3.0/v2.1 的字段和格式。
- [isaac_script_editor_diagnostics/README.md](isaac_script_editor_diagnostics/README.md)：面向 Isaac Script Editor 诊断脚本，说明 joint1-7 drive 和 Action Graph 检查脚本。
- [graph/ALL_Graph.md](graph/ALL_Graph.md)：Isaac Action Graph 全量导出。
- [graph/Gripper_Control_Graph.md](graph/Gripper_Control_Graph.md)：夹爪 Action Graph 导出。
- [graph/scene_structure.md](graph/scene_structure.md)：Stage、Robot、Joint、Drive、Camera、Physics Scene 结构导出。
- [graph/gripper_full_diagnosis.md](graph/gripper_full_diagnosis.md)：PGIA 夹爪完整诊断。
- [gripper_notes.md](gripper_notes.md)：历史接手说明，包含夹爪控制和 OCS2 链路背景。
- [ROS2_LOG_CLEANUP_PLAN.md](ROS2_LOG_CLEANUP_PLAN.md)：`~/ros2_log` 整理计划，属于辅助维护文档。

## 6. 数据与回放原则

原始示教统一写入：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.diagnostics.csv
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.metadata.json
```

启用 `--record-cameras` 时，相机输出写入：

```text
trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS/camera/
```

示教成功后默认执行 `trace_cleaning/scripts/process_sim1_episode.py`：

```text
trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned.csv
trace_data/cleaned/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned/camera/
trace_data/lerobot_v30/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned/
trace_data/lerobot_v21/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS_cleaned/  # 仅显式启用
```

`./game_control.sh demo` 默认优先回放：

```text
trace_data/raw/manual_ocs2_keyboard_trace_latest.csv
```

指定文件回放：

```bash
TRACE=trace_data/cleaned/manual_ocs2_keyboard_trace_20260529_154535_cleaned.csv ./game_control.sh demo
```

清洗和 LeRobot 转换细节见 [TRACE_DATA_FORMAT.md](TRACE_DATA_FORMAT.md)。

底盘独立记录写入：

```text
trace_data/base_raw/base_drive_trace_YYYYmmdd_HHMMSS.csv
trace_data/base_raw/base_drive_trace_latest.csv
```

底盘命令通过 `/sim1/base_diff_cmd` 的 `geometry_msgs/msg/Twist` 发布，只使用 `linear.x` 和 `angular.z`。Isaac Sim `Base_Drive_Graph` 当前实际使用的底盘 ScriptNode 是 `base_control/script_node.py`；若场景换成 `${PROJECT_ROOT}/data/ros2_log/scripts/base_control/script_node_base_arc_debug_v2.py`，则会额外发布 `/sim1/base_drive_debug` JSON 调试话题。`base_drive_recorder.py` 和 `keyboard_ocs2_gripper_teleop.py` 已订阅该 debug topic，存在数据时会记录 ScriptNode 内部 arc 状态和轮速命令。`./game_control.sh base-record` 启动独立记录，`./game_control.sh base-path-demo` 执行前进/圆弧转弯测试路径。`./game_control.sh teach --input-mode pygame` 默认启用底盘片选控制，按 `3` 选中底盘后方向键生效；可用 `--disable-base-motion` 关闭。

## 7. 异常排查入口

力矩突然消失、右臂发散、target-current 误差持续扩大时，优先检查：

```text
trace_data/raw/*.diagnostics.csv
right_target_current_norm
right_orientation_error_deg
right_joint1_position
right_joint1_effort
right_joint2/3/4_position
ocs2_sqp_not_converged_count
ocs2_cost_explosion_count
fsm_is_ocs2
```

Isaac 侧 drive、Action Graph、joint order 排查：

```text
isaac_script_editor_diagnostics/
graph/ALL_Graph.md
graph/scene_structure.md
```

reset 相关问题优先看 [GAME_GUIDE.md](GAME_GUIDE.md) 的常见问题：

```text
/reset_simulation 服务不可用
simulation_interfaces 缺失
Isaac reset 后 OCS2 target/current pose 未同步
OCS2 controller/FSM 没收到 HOME
```

## 8. 修改原则

- `./game_control.sh teach` 默认保持最小行为：不自动启用 workspace clamp、lag gate、姿态 gate 或 watchdog。
- 安全门限和 workspace clamp 通过命令行显式开启，便于区分“原始行为”和“受保护示教”。
- 正式示教优先用 `--input-mode pygame`，终端模式保留给 SSH、无图形环境和简单调试。
- 新增数据字段时同步更新 [TRACE_DATA_FORMAT.md](TRACE_DATA_FORMAT.md)。
- 新增操作流程或命令时同步更新 [GAME_GUIDE.md](GAME_GUIDE.md)。
