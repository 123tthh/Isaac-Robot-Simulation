# 第三阶段任务规划：小样本示教 + 随机化 + Isaac Sim 自动化测试 + 数据生成训练

> 用于开启新对话，使 LLM / Codex 快速理解当前项目背景、已完成能力、当前目标和下一步实现路线。  
> 搭配 `~/ros2_ws/CLAUDE.md`、`~/Trajectory/SIM1/PROJECT_OVERVIEW.md`、`TRACE_DATA_FORMAT.md`、`GAME_GUIDE.md`、`ALL_Graph.md`、`Gripper_Control_Graph.md`、`scene_structure.md` 使用。

---

## 一、项目背景概要

### 1.1 项目目标

R1 轮式双臂机器人在 Isaac Sim 5.1 中完成传送带托盘放置 / 抓取 / 递交任务。当前路线从“只录制少量人工示教轨迹”升级为：

```text
少量高质量人工示教
  → 轨迹清洗与阶段切分
  → 关键帧参数化
  → 轨迹 / 场景 / 物理参数随机化
  → Isaac Sim 自动批量回放与测试
  → 读取 Isaac Sim API / ROS2 topic 自动判定成功失败
  → 生成数百条可筛选训练数据
  → LeRobot / BC / ACT / Diffusion Policy / RL warm-start / HIL-SERL
```

核心判断：**可行，但扩增数据不能直接当专家数据，必须经过 Isaac Sim 真实物理回放和自动化成功判定。** 失败样本也要保留，但应进入 failure / negative dataset，用于 reward classifier、success classifier、critic 或失败案例分析，不能混入 expert demo。

---

### 1.2 阶段划分与当前状态

| 阶段 | 目标 | 当前状态 |
|------|------|----------|
| 阶段1 | OCS2 MPC ↔ Isaac Sim 控制闭环 | ✅ 已完成 |
| 阶段2 | 夹爪控制、碰撞、轨迹示教、回放、诊断链路 | 已打通，仍需持续验证夹持稳定性 |
| 阶段3 | 小样本示教 + 随机化 + Isaac Sim 自动化测试 + 数据生成 | 🔴 **当前重点** |
| 阶段4 | 数据集训练：BC / ACT / Diffusion Policy / RL warm-start / HIL-SERL | ⏳ 待开始 |
| 阶段5 | Sim2Real：实机验证、误差分析、少量真机数据微调 | ⏳ 待开始 |

---

### 1.3 已完成的关键能力

#### 1.3.1 OCS2 → ROS2 → Isaac Sim 控制闭环

已经实现：

- ✅ RViz2 Interactive Marker / 键盘示教目标 → OCS2 target
- ✅ OCS2 MPC 控制 14 个双臂关节：`left_joint1-7 + right_joint1-7`
- ✅ `topic_based_ros2_control/TopicBasedSystem` 作为 Isaac Sim ↔ ros2_control 桥接
- ✅ `/arm_joint_cmd` 驱动 Isaac Sim `Arm_Control_Graph`
- ✅ Isaac Sim 物理状态经 `/isaac_joint_states` / `/joint_states` 回传
- ✅ `/left_current_pose`、`/right_current_pose` 可用于 target-current 误差诊断
- ✅ `/clock`、`/odom`、TF、相机话题可支撑录制和回放

当前推荐状态链路：

```text
Isaac Sim State_Telemetry_Graph
  → /isaac_joint_states
  → joint_state_fill.py
  → /joint_states
  → robot_state_publisher / ros2_control / OCS2 / RViz
```

控制链路：

```text
keyboard_ocs2_gripper_teleop.py / RViz Interactive Marker
  → /left_target/stamped, /right_target/stamped
  → ocs2_arm_controller
  → topic_based_ros2_control
  → /arm_joint_cmd
  → Isaac Sim Arm_Control_Graph
  → Articulation Controller
  → R1 双臂运动
```

---

#### 1.3.2 SIM1 示教 / 回放 / 数据处理工程

当前主要工程目录：

```text
~/Trajectory/SIM1
```

核心能力：

- ✅ `./game_control.sh check`：检查关键 ROS2 topic / service / Isaac 连接状态
- ✅ `./game_control.sh teach`：键盘 / pygame 多键示教，发布 OCS2 target，并同步记录轨迹、诊断、夹爪、关节、RGB+D 相机数据
- ✅ `./game_control.sh demo`：回放最近一次 OCS2 target trace
- ✅ `replay_lerobot_episode.py`：LeRobot 关节轨迹直连回放
- ✅ `tools/camera_recorder.py`：独立 ROS2 相机 recorder，可由示教器 subprocess 启动
- ✅ `trace_data/raw/`：原始示教 CSV、diagnostics CSV、metadata、label、原始 camera run
- ✅ `trace_data/cleaned/`：清洗后轨迹和按同一时间区间裁剪后的 camera run
- ✅ `trace_data/lerobot_v30/`：LeRobotDataset v3.0 本地数据集输出
- ✅ `trace_data/lerobot_v21/`：LeRobotDataset v2.1 本地数据集输出
- ✅ diagnostics CSV 记录 target-current 误差、姿态误差、OCS2 异常、关节位置、关节 effort、相机 recorder 状态和 WARNING

推荐正式示教入口：

```bash
cd ~/Trajectory/SIM1
source ~/ros2_ws/install/setup.bash
./game_control.sh check
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

该命令是当前正式采集基线：**轨迹 + diagnostics + 夹爪 + 关节位置/effort + 3 路 RGB 视频 + 3 路 depth npy**。正式采集不使用 `--camera-rgb-only`，避免只得到“轨迹 + RGB”而缺少 depth。

---

#### 1.3.3 SIM1 接口、架构与数据框架

SIM1 的定位不是单个示教脚本，而是 Phase3 数据工厂的本地接口层：

```text
Isaac Sim / ROS2 / OCS2 实时控制
  -> keyboard_ocs2_gripper_teleop.py
  -> trace_data/raw/<run_id>.csv + diagnostics + metadata + camera/
  -> trace_cleaning/scripts/clean_sim1_episode.py
  -> trace_data/cleaned/<run_id>_cleaned.csv + camera/
  -> convert_ocs2_trace_to_lerobot_v30.py
  -> convert_ocs2_trace_to_lerobot_v21.py
  -> trace_data/lerobot_v30/<run_id>_cleaned/
  -> trace_data/lerobot_v21/<run_id>_cleaned/
```

主要控制接口：

```text
/left_target/stamped              geometry_msgs/PoseStamped
/right_target/stamped             geometry_msgs/PoseStamped
/left_current_pose                geometry_msgs/PoseStamped
/right_current_pose               geometry_msgs/PoseStamped
/left_gripper_controller/commands std_msgs/Float64MultiArray
/right_gripper_controller/commands std_msgs/Float64MultiArray
/joint_states                     sensor_msgs/JointState
/isaac_joint_states               sensor_msgs/JointState
/fsm_command                      std_msgs/Int32
```

主要相机接口：

```text
/head_cam/color/image_raw          rgb8, 640x480
/head_cam/depth/image_rect_raw     32FC1, 640x480
/left_cam/color/image_raw          rgb8, 640x480
/left_cam/depth/image_rect_raw     32FC1, 640x480
/right_cam/color/image_raw         rgb8, 640x480
/right_cam/depth/image_rect_raw    32FC1, 640x480
```

数据保存原则：

```text
raw      保留原始采集，不覆盖，不手工剪裁
cleaned  删除无控制/无动作观察段，并同步裁剪视频和 depth
lerobot  v3.0 与 v2.1 都从 cleaned 数据导出
sync     轨迹和图像按 ROS timestamp / wall timestamp 对齐，不按帧序号硬对齐
label    成功/失败原因记录在 label.md、diagnostics、后续 failure json 中
```

SIM1 的目的：先建立可靠、可追溯、可回放、可转换的小样本专家数据闭环，再在此基础上做随机化、自动评测和数据扩增。

---

#### 1.3.4 夹爪控制当前约定

当前夹爪控制应以 **PGIA v4.5 位置控制模式** 为准，不再按旧版 Mimic / 单主动描述理解。

关键点：

```text
left_PGIA_joint1 + left_PGIA_joint2   双主动位置控制
right_PGIA_joint1 + right_PGIA_joint2 双主动位置控制
```

命令 topic：

```text
/left_gripper_controller/commands   std_msgs/msg/Float64MultiArray
/right_gripper_controller/commands  std_msgs/msg/Float64MultiArray
```

命令语义：

```text
data: [1.0]  → OPEN
data: [0.0]  → CLOSE / HOLD
```

Graph 期望连接：

```text
sub_left_gripper.outputs:data
  → script_left_gripper.inputs:input_double_array
  → script_left_gripper.outputs:position_cmds
  → artic_left_gripper.inputs:positionCommand

sub_right_gripper.outputs:data
  → script_right_gripper.inputs:input_double_array
  → script_right_gripper.outputs:position_cmds
  → artic_right_gripper.inputs:positionCommand
```

验证命令：

```bash
ros2 topic pub --once /left_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic pub --once /left_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"

ros2 topic pub --once /right_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [1.0]}"
ros2 topic pub --once /right_gripper_controller/commands \
  std_msgs/msg/Float64MultiArray "{data: [0.0]}"
```

判断标准：

- `[1.0]` 能稳定打开
- `[0.0]` 能稳定闭合 / HOLD
- 有托盘时 HOLD 后托盘不明显滑落
- 翻转 / 平移时托盘不因瞬时速度误判而被释放
- `./game_control.sh teach` 中 Space / Z/X / N/M / C/V 与直接 topic pub 表现一致

---

### 1.4 机器人关节配置

| 关节组 | 关节名 | 数量 | 控制方式 | 当前状态 |
|--------|--------|------|----------|----------|
| 左臂 | `left_joint1-7` | 7 | OCS2 MPC 位置控制 | ✅ 已通 |
| 右臂 | `right_joint1-7` | 7 | OCS2 MPC 位置控制 | ✅ 已通 |
| 左夹爪 | `left_PGIA_joint1/2` | 2 | v4.5 双主动位置控制，离散命令 0/1 | 🟡 需持续验证夹持稳定性 |
| 右夹爪 | `right_PGIA_joint1/2` | 2 | v4.5 双主动位置控制，离散命令 0/1 | 🟡 需持续验证夹持稳定性 |
| 躯干 | `waist_joint`, `body_joint` | 2 | 暂不主动规划 | — |
| 轮子 | `wheel_L/R`, `caster_*` | 6 | 阶段 B 底盘移动 / 后续扩展 | ⏳ |

---

### 1.5 Isaac Sim Action Graph 架构

```text
Arm_Control_Graph:
  订阅 /arm_joint_cmd
  → 驱动 left_joint1-7 + right_joint1-7

Gripper_Control_Graph:
  订阅 /left_gripper_controller/commands
  订阅 /right_gripper_controller/commands
  → Script Node v4.5
  → positionCommand 驱动 PGIA_joint1/2

State_Telemetry_Graph:
  发布 /clock
  发布 /isaac_joint_states
  → joint_state_fill.py 补齐后输出 /joint_states

Camera_Publish_Graph:
  发布 head / left wrist / right wrist RGB-D 话题
```

---

### 1.6 关键目录与文件

```text
~/ros2_ws/src/robot/                    ← 包名: r1_description
  urdf/r1_fixed.urdf
  xacro/ros2_control/robot.xacro
  config/ocs2/task.info
  config/ros2_control/ocs2_controllers.yaml
  launch/ocs2_isaac.launch.py
  scripts/joint_state_fill.py
  scripts/odom_to_tf.py

~/ros2_ws/src/arms_ros2_control/
  command/arms_target_manager/
  controller/ocs2_arm_controller/
  hardwares/topic_based_ros2_control/

~/Trajectory/SIM1/
  game_control.sh
  keyboard_ocs2_gripper_teleop.py
  tools/camera_recorder.py
  replay_ocs2_target_trace.py
  replay_lerobot_episode.py
  TRACE_DATA_FORMAT.md
  GAME_GUIDE.md
  PROJECT_OVERVIEW.md
  Phase3_TaskPlan_updated.md
  trace_data/raw/
  trace_data/cleaned/
  trace_data/generated/
  trace_data/generated_success/
  trace_data/generated_failure/
  trace_data/lerobot_v30/
  trace_data/lerobot_v21/
  trace_cleaning/scripts/clean_sim1_episode.py
  trace_cleaning/scripts/process_sim1_episode.py
  trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py
  trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py
  isaac_script_editor_diagnostics/
  graph/ALL_Graph.md
  graph/Gripper_Control_Graph.md
  graph/scene_structure.md
```

---

## 二、当前阶段核心任务

当前阶段不再只是“录几条轨迹”，而是要搭建一个可重复运行的数据生成闭环。

| 任务 | 名称 | 目标 |
|------|------|------|
| 任务 A | 高质量小样本示教 | 录制 5~20 条干净成功轨迹，覆盖放置、递交、抓取关键阶段 |
| 任务 B | 轨迹清洗与阶段切分 | 删除静止段 / 发散段 / reset 段，切分为关键帧和阶段 |
| 任务 C | 轨迹与场景随机化 | 生成大量候选 episode，但不直接当成功数据 |
| 任务 D | Isaac Sim 自动批量测试 | reset → 回放 → 读取状态 → 自动判定 success / failure |
| 任务 E | 数据集构建 | 成功样本进入 expert demo，失败样本进入 negative dataset |
| 任务 F | 训练与评估 | 先 BC / ACT / Diffusion Policy，再考虑 RL / HIL-SERL |

---

## 三、任务 A：高质量小样本示教

### 3.1 目标

先人工录制少量但干净的成功样本。推荐数量：

```text
最小可行：5 条完整成功轨迹
较稳版本：10~20 条完整成功轨迹
不要一开始追求数量，先追求可回放、可清洗、可判定。
```

### 3.2 示教原则

- 每条轨迹必须能在 Isaac Sim 中完整回放
- 正式采集必须同时记录 RGB+D，不只采轨迹，也不只采轨迹 + RGB
- target-current 误差不能持续扩大
- 不要把长时间无动作观察段混入训练数据
- 夹爪开闭事件必须准确记录
- 每帧相机数据必须保存 ROS timestamp 和 wall timestamp，后续按 timestamp 对齐
- 每个阶段至少标记关键帧
- 失败轨迹不要删除，应单独标注失败原因

### 3.3 建议录制阶段

```text
A1_right_place        右臂将托盘1放到输入传送带
A2_left_to_handover   左臂将托盘2送到递交区
A3_dual_handover      左右臂递交托盘
A4_right_place_2      右臂放置托盘2
C1_right_grasp_1      右臂从输出传送带抓托盘1
C2_right_to_left      右臂递交给左臂
C3_right_grasp_2      右臂抓托盘2
```

### 3.4 原始数据记录字段

必须保留：

```text
wall_time
ros_time_sec
selected
event
left_target_xyz / right_target_xyz
left_target_xyzw / right_target_xyzw
left_current_xyz / right_current_xyz
left_current_xyzw / right_current_xyzw
left_gripper_cmd / right_gripper_cmd
left_gripper_closed / right_gripper_closed
joint_names / joint_positions
effort_joint_names / joint_efforts
```

原始相机数据必须保留：

```text
trace_data/raw/<run_id>/camera/videos/head_rgb.mp4
trace_data/raw/<run_id>/camera/videos/left_rgb.mp4
trace_data/raw/<run_id>/camera/videos/right_rgb.mp4
trace_data/raw/<run_id>/camera/depth/head_depth/*.npy
trace_data/raw/<run_id>/camera/depth/left_depth/*.npy
trace_data/raw/<run_id>/camera/depth/right_depth/*.npy
trace_data/raw/<run_id>/camera/camera_timestamps.csv
trace_data/raw/<run_id>/camera/summary.json
```

诊断 CSV 必须保留：

```text
left_target_current_norm
right_target_current_norm
left_orientation_error_deg
right_orientation_error_deg
fsm_is_ocs2
blocked_translation_count
blocked_rotation_count
workspace_clamp_count
hard_lag_block_count
watchdog_freeze_count
ocs2_sqp_not_converged_count
ocs2_cost_explosion_count
left_joint1_position ... left_joint7_position
right_joint1_position ... right_joint7_position
left_joint1_effort ... left_joint7_effort
right_joint1_effort ... right_joint7_effort
camera_recorder_status
camera_output_dir
camera_warning
```

---

## 四、任务 B：轨迹清洗与阶段切分

### 4.1 清洗目标

把人工示教轨迹变成可训练、可扩增、可回放的干净片段。

需要删除：

```text
长时间 tick 但 target / current 基本不动的观察段
quit / keyboard_interrupt / reset_after_task 行
target-current 严重发散段
OCS2 非 OCS2 模式段
明显误操作段
夹爪命令抖动段
```

需要保留：

```text
有效 target 运动段
关键夹爪开闭事件
接近 / 抓取 / 提起 / 放置 / 释放 / 退出动作
失败片段的 failure label
```

当前 SIM1 清洗入口：

```bash
cd ~/Trajectory/SIM1
./trace_cleaning/scripts/clean_sim1_episode.py \
  trace_data/raw/manual_ocs2_keyboard_trace_YYYYmmdd_HHMMSS.csv
```

该脚本会：

```text
1. 复用 clean_ocs2_trace.py 的动作区间保留逻辑
2. 删除无控制、无动作、只观察 Isaac Sim 视角的长时间段
3. 压缩 cleaned CSV 的 wall_time / ros_time_sec
4. 按同一保留区间裁剪 RGB mp4
5. 按 timestamp 筛选复制 depth npy
6. 重写 cleaned/camera/camera_timestamps.csv
```

注意：视频和轨迹不要按帧序号强行对齐。清洗和转换都应使用 `camera_timestamps.csv` 中的 ROS timestamp / wall timestamp。

### 4.2 阶段切分输出

建议每条 episode 生成：

```text
trace_data/cleaned/<run_id>_cleaned.csv
trace_data/cleaned/<run_id>_cleaned.report.md
trace_data/cleaned/<run_id>_cleaned/camera/
trace_data/raw/<run_id>.label.md
trace_data/raw/<run_id>.diagnostics.csv
trace_data/raw/<run_id>.metadata.json
episode_xxx.keyframes.yaml             # 后续阶段切分输出
episode_xxx.failure.json               # 仅失败样本需要
```

关键帧 YAML 示例：

```yaml
episode_id: episode_0007
task: right_place_tray
success: true
keyframes:
  - name: approach
    t: 1.20
    active_arm: right
    gripper: closed
  - name: pre_place
    t: 3.45
    active_arm: right
    gripper: closed
  - name: release
    t: 4.10
    active_arm: right
    gripper: open
  - name: retreat
    t: 5.30
    active_arm: right
    gripper: open
```

---

## 五、任务 C：轨迹 / 场景 / 物理随机化

### 5.1 核心原则

不要直接对每一帧关节角随机加噪声。托盘抓放是接触任务，直接扰动关节轨迹很容易破坏抓取、释放和接触时序。

推荐随机化对象：

```text
关键帧目标 pose
托盘初始 pose
放置目标 pose
执行速度 / phase duration
夹爪开闭时刻
物理参数小范围扰动
视觉参数扰动
```

---

### 5.2 轨迹关键帧随机化

放置任务：

```text
place_target.x += uniform(-0.02, +0.02) m
place_target.y += uniform(-0.02, +0.02) m
place_target.z += uniform(-0.005, +0.010) m
place_target.yaw += uniform(-5°, +5°)
```

抓取任务：

```text
tray_initial.x += uniform(-0.015, +0.015) m
tray_initial.y += uniform(-0.015, +0.015) m
tray_initial.yaw += uniform(-5°, +5°)
approach_height += uniform(-0.01, +0.02) m
```

递交任务：

```text
handover_center.x/y/z += small uniform noise
receiver_pregrasp_pose += small uniform noise
gripper_close_time += uniform(-0.15, +0.15) s
```

---

### 5.3 时间随机化

```text
phase_duration *= uniform(0.8, 1.2)
wait_time *= uniform(0.5, 1.5)
gripper_event_time += uniform(-0.10, +0.20) s
```

注意：夹爪闭合不能早于到达 pregrasp，也不能晚于 lift。需要设置事件顺序约束。

---

### 5.4 物理参数随机化

前期只做小范围：

```text
tray_mass *= uniform(0.9, 1.1)
tray_static_friction *= uniform(0.8, 1.2)
tray_dynamic_friction *= uniform(0.8, 1.2)
gripper_static_friction *= uniform(0.8, 1.2)
gripper_dynamic_friction *= uniform(0.8, 1.2)
joint_damping *= uniform(0.8, 1.2)
```

不要一开始使用极端摩擦随机化。夹爪 / 托盘接触已经是当前系统的敏感点，随机范围过大会产生大量不可用失败数据。

---

### 5.5 视觉随机化

视觉策略训练前再开启：

```text
light_intensity *= uniform(0.7, 1.3)
material_color jitter
camera_noise small
background texture small randomization
RGB crop / resize jitter
```

建议顺序：

```text
先只训练 state/action 轨迹策略
再加入 RGB-D / VLA / visual policy
```

---

## 六、任务 D：Isaac Sim 自动化批量测试

### 6.1 目标

每条随机化候选轨迹都必须经过真实仿真执行：

```text
reset simulation
apply randomized scene / task config
replay candidate trajectory
monitor robot / tray / gripper / collision state
judge success or failure
save episode result
```

---

### 6.2 自动测试脚本建议

建议新增：

```text
~/Trajectory/SIM1/tools/trajectory_augmenter.py
~/Trajectory/SIM1/tools/isaac_batch_tester.py
~/Trajectory/SIM1/tools/success_judge.py
~/Trajectory/SIM1/tools/export_generated_lerobot.py
```

最小数据流：

```text
trace_data/cleaned/*_cleaned.csv + cleaned camera/
  → trajectory_augmenter.py
  → trace_data/generated/candidate_*.yaml
  → isaac_batch_tester.py
  → trace_data/generated_success/*.csv
  → trace_data/generated_failure/*.csv + failure_reason.json
  → export_generated_lerobot.py
  → trace_data/lerobot_v30_augmented/
  → trace_data/lerobot_v21_augmented/
```

---

### 6.3 自动判定信号来源

ROS2 topic 层：

```text
/joint_states
/isaac_joint_states
/left_current_pose
/right_current_pose
/odom
/left_gripper_controller/commands
/right_gripper_controller/commands
/fsm_command
```

Isaac Sim API 层：

```text
托盘 world pose
托盘 linear / angular velocity
托盘是否在目标区域
托盘是否掉落
夹爪 link pose
夹爪与托盘接触状态
异常碰撞 / 穿模
场景 reset 是否完成
```

---

### 6.4 成功判定规则

#### 放置成功

```python
success_place = (
    tray_pos in target_region
    and abs(tray_z - conveyor_height) < z_threshold
    and abs(tray_roll) < roll_threshold
    and abs(tray_pitch) < pitch_threshold
    and norm(tray_linear_velocity) < velocity_threshold
    and gripper_is_open
    and not robot_self_collision
    and not tray_fallen
    and not ocs2_diverged
)
```

#### 抓取成功

```python
success_grasp = (
    tray_relative_pose_to_gripper_is_stable
    and tray_lift_height > lift_threshold
    and gripper_is_closed
    and tray_not_slipping
    and norm(tray_linear_velocity_relative_to_gripper) < threshold
    and not robot_self_collision
    and not ocs2_diverged
)
```

#### 递交成功

```python
success_handover = (
    receiver_gripper_closed
    and sender_gripper_open
    and tray_attached_to_receiver_motion
    and tray_not_fallen
    and left_right_gripper_distance_safe
    and not severe_collision
)
```

---

### 6.5 failure reason 标注

失败样本不要丢弃。建议保存：

```json
{
  "episode_id": "candidate_0031",
  "success": false,
  "failure_reason": "tray_slip_after_lift",
  "stage": "C1_right_grasp_1",
  "metrics": {
    "max_target_current_error": 0.132,
    "tray_final_z": 0.41,
    "tray_velocity_norm": 0.36,
    "right_gripper_closed": true
  }
}
```

failure reason 枚举：

```text
ocs2_diverged
target_current_lag_too_large
gripper_failed_to_close
gripper_failed_to_open
tray_slip_after_lift
tray_fallen
place_out_of_region
tray_unstable_after_release
robot_self_collision
left_right_arm_collision
scene_reset_failed
unknown
```

---

## 七、任务 E：数据集构建

### 7.1 数据分类

```text
raw/                 → 原始采集证据链，保留轨迹、diagnostics、label、metadata、RGB-D camera run，不直接训练
cleaned/             → 人工示教清洗后样本，成功轨迹可作为第一批 expert demo
generated_success/   → 自动随机化并经 Isaac Sim 验证成功的扩增 expert demo
generated_failure/   → negative dataset，可用于 reward classifier / critic / failure analysis
generated_all/       → 原始候选和测试日志，不直接训练
```

### 7.2 LeRobotDataset v3.0 / v2.1 输出建议字段

当前 SIM1 已支持从 cleaned 数据直接导出：

```bash
./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v30.py \
  trace_data/cleaned/<run_id>_cleaned.csv \
  --output trace_data/lerobot_v30/<run_id>_cleaned

./trace_cleaning/scripts/convert_ocs2_trace_to_lerobot_v21.py \
  trace_data/cleaned/<run_id>_cleaned.csv \
  --output trace_data/lerobot_v21/<run_id>_cleaned
```

或一键执行 raw → cleaned → v3.0 → v2.1：

```bash
./trace_cleaning/scripts/process_sim1_episode.py \
  trace_data/raw/<run_id>.csv
```

v3.0 是 file-based：

```text
trace_data/lerobot_v30/<run_id>_cleaned/
  data/chunk-000/file-000.parquet
  videos/observation.images.head/chunk-000/file-000.mp4
  videos/observation.images.left_wrist/chunk-000/file-000.mp4
  videos/observation.images.right_wrist/chunk-000/file-000.mp4
  meta/info.json
  meta/stats.json
  meta/tasks.parquet
  meta/episodes/chunk-000/file-000.parquet
```

v2.1 是 episode-based：

```text
trace_data/lerobot_v21/<run_id>_cleaned/
  data/chunk-000/episode_000000.parquet
  videos/chunk-000/observation.images.head/episode_000000.mp4
  videos/chunk-000/observation.images.left_wrist/episode_000000.mp4
  videos/chunk-000/observation.images.right_wrist/episode_000000.mp4
  meta/info.json
  meta/tasks.jsonl
  meta/episodes.jsonl
  meta/episodes_stats.jsonl
```

```text
observation.state                 14 维双臂关节角 + 左右夹爪闭合状态
observation.state.pose            左右 OCS2 target pose + 左右夹爪闭合状态
observation.effort                14 维双臂关节 effort
observation.images.head           头部 RGB 视频
observation.images.left_wrist     左腕 RGB 视频
observation.images.right_wrist    右腕 RGB 视频
action                            下一帧 observation.state
action.pose                       下一帧 observation.state.pose
observation.six_force             预留，可为空或后续接力觉
observation.six_force_tcp         预留，可为空或后续接 TCP 力觉
timestamp
frame_index
episode_index
index
task_index
success
failure_reason
phase
```

depth 数据当前保留在 raw/cleaned camera 目录中，用于后续 RGB-D/VLA 或点云处理；现有 LeRobot 转换先把 3 路 RGB 作为 video feature 写入，depth npy 通过 cleaned camera run 保持可追溯。

### 7.3 训练数据使用原则

| 数据类型 | 用途 | 是否进入 expert demo |
|----------|------|----------------------|
| 人工成功轨迹 | BC / imitation 初始数据 | ✅ |
| 自动生成且仿真成功 | 扩增 expert demo | ✅ |
| 自动生成但失败 | reward classifier / critic / 失败分析 | ❌ |
| 物理异常 / reset 失败 | 调试，不训练 | ❌ |
| OCS2 发散轨迹 | 安全边界学习 / 失败分析 | ❌ |

---

## 八、任务 F：训练路线

### 8.1 第一阶段：非视觉 state/action BC

先用低维状态训练，避免一开始被视觉问题干扰。

输入：

```text
14 维关节角
左右夹爪状态
左右末端 pose
目标 pose / phase id
历史 action
```

输出：

```text
下一步 joint target
或下一步 ee_delta + gripper command
```

目标：

```text
验证数据质量和动作可回放性
```

---

### 8.2 第二阶段：ACT / Diffusion Policy

适合处理长时序、多阶段、接触任务。

建议：

```text
先单臂 right_place_tray
再 right_grasp_tray
再 dual_handover
最后完整 A-B-C 循环
```

---

### 8.3 第三阶段：视觉策略 / VLA

加入：

```text
head RGB-D
left wrist RGB-D
right wrist RGB-D
phase language instruction
task token
```

语言指令示例：

```text
"place the tray onto the input conveyor"
"grasp the tray from the output conveyor"
"handover the tray from right gripper to left gripper"
```

---

### 8.4 第四阶段：RL / HIL-SERL

等自动评测稳定后再做：

```text
success classifier
reward classifier
BC warm-start policy
online RL fine-tune
human intervention correction
```

不要在自动评测不稳定时直接上 RL。否则调不清是 policy 差、reward 差、物理接触差，还是 reset / 判定脚本有问题。

---

## 九、最小可行版本 V0

V0 只做一个子任务：**右臂放置托盘到输入传送带**。

### 9.1 V0 输入

```text
5 条右臂放置成功人工示教轨迹
每条轨迹同步采集 3 路 RGB + 3 路 depth
每条轨迹必须能通过 ./game_control.sh demo 回放
每条轨迹必须能通过 process_sim1_episode.py 生成 cleaned、lerobot_v30、lerobot_v21
```

### 9.2 V0 随机化

```text
place_target x/y ±2 cm
place_target z -0.5 cm / +1 cm
place_target yaw ±5°
phase duration 0.8~1.2×
gripper release time ±0.1 s
```

### 9.3 V0 自动测试

```text
每条人工轨迹生成 20 个候选
5 × 20 = 100 条 candidate
Isaac Sim 批量回放
自动判定 place success
保存 success / failure
```

### 9.4 V0 通过标准

```text
候选轨迹可自动 reset / replay / judge
成功样本能稳定导出 LeRobot v3.0
成功样本能稳定导出 LeRobot v2.1
失败样本有明确 failure_reason
至少获得 50 条以上成功扩增样本
随机种子固定时结果可复现
```

---

## 十、启动流程

### 10.1 Isaac Sim + OCS2 启动

```bash
# Terminal 1: Isaac Sim 打开 Stage 并 Play

# Terminal 2: 启动 OCS2 闭环
cd ~/ros2_ws
source install/setup.bash
ros2 launch r1_description ocs2_isaac.launch.py

# Terminal 3: 检查 SIM1 工程
cd ~/Trajectory/SIM1
source ~/ros2_ws/install/setup.bash
./game_control.sh check
```

### 10.2 基础话题检查

```bash
ros2 topic hz /clock
ros2 topic hz /isaac_joint_states
ros2 topic hz /joint_states
ros2 topic echo --once /left_current_pose
ros2 topic echo --once /right_current_pose
ros2 topic info -v /head_cam/color/image_raw
ros2 topic info -v /head_cam/depth/image_rect_raw
ros2 topic info -v /left_cam/color/image_raw
ros2 topic info -v /left_cam/depth/image_rect_raw
ros2 topic info -v /right_cam/color/image_raw
ros2 topic info -v /right_cam/depth/image_rect_raw
ros2 control list_controllers
```

期望：

```text
joint_state_broadcaster active
ocs2_arm_controller active
/joint_states publisher count = 1
/isaac_joint_states publisher count = 1
head/left/right RGB topic publisher count >= 1
head/left/right depth topic publisher count >= 1
```

### 10.3 切入 OCS2

```bash
ros2 topic pub --once /fsm_command std_msgs/msg/Int32 "{data: 3}"
```

### 10.4 录制示教

```bash
cd ~/Trajectory/SIM1
source ~/ros2_ws/install/setup.bash
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

该命令默认在录制成功后执行 `trace_cleaning/scripts/process_sim1_episode.py`，生成 cleaned、LeRobotDataset v3.0、LeRobotDataset v2.1。若只想保留完整 raw RGB+D 而暂不后处理，可加：

```bash
SIM1_PROCESS_AFTER_TEACH=0 ./game_control.sh teach --input-mode pygame \
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

### 10.5 回放示教

```bash
./game_control.sh demo
```

### 10.6 后续建议新增命令

```bash
# 生成候选随机化 episode
python3 tools/trajectory_augmenter.py \
  --input trace_data/cleaned/right_place_success_*.csv \
  --num-variants 20 \
  --output trace_data/generated/

# 批量测试候选 episode
python3 tools/isaac_batch_tester.py \
  --input trace_data/generated/ \
  --success-dir trace_data/generated_success/ \
  --failure-dir trace_data/generated_failure/

# 导出增强 LeRobot 数据集
python3 tools/export_generated_lerobot.py \
  --input trace_data/generated_success/ \
  --output trace_data/lerobot_v30_augmented/

python3 tools/export_generated_lerobot.py \
  --input trace_data/generated_success/ \
  --output trace_data/lerobot_v21_augmented/
```

---

## 十一、实施顺序

```text
Week 1: 数据基础闭环
  Day 1: 确认 OCS2 / joint_states / gripper / reset / camera RGB-D / demo 回放稳定
  Day 2: 用正式 RGB+D 命令录制 5 条 right_place_tray 成功轨迹
  Day 3: 用 process_sim1_episode.py 生成 cleaned、lerobot_v30、lerobot_v21，并补 keyframes.yaml / label.md
  Day 4: 完成 trajectory_augmenter.py 最小版本
  Day 5: 完成 success_judge.py 的 place 判定

Week 2: 自动化批量测试
  Day 6: 完成 isaac_batch_tester.py reset / replay / judge 主循环
  Day 7: 生成 100 条 candidate 并批量测试
  Day 8: 保存 success / failure / failure_reason
  Day 9: 导出 LeRobot v3.0 / v2.1 augmented dataset
  Day 10: 直连回放验证 generated_success 样本

Week 3: 扩展到抓取和递交
  Day 11-12: right_grasp_tray 数据生成
  Day 13-14: right_to_left_handover 数据生成
  Day 15: 合并 task_index / phase / success / failure_reason

Week 4: 训练准备
  Day 16-17: state/action BC baseline
  Day 18-19: ACT / Diffusion Policy baseline
  Day 20: 评估失败样本，反向改进随机化范围和成功判定规则
```

---

## 十二、已解决问题清单（背景保留）

阶段1 OCS2 ↔ Isaac Sim 闭环已解决的问题：

1. URDF 手臂 Link 无惯性数据
2. trunk_link 惯量单位错误
3. D455 相机惯量偏大
4. waist/body_joint effort=0
5. 关节位置控制飞出（Damping 不足）
6. left/right_flange_joint 重复 origin
7. head_camera/lidar joint → fixed
8. use_sim_time 导致 spawner 超时
9. `libarms_target_manager.so` 找不到
10. URDF mesh 路径硬编码
11. RViz TF 重映射导致全断
12. TF 双源冲突（Isaac Sim + robot_state_publisher）
13. `odom_to_tf.py` 时间戳 sec=0
14. world→base_link 断裂
15. `robot.xacro` 加载 mock 而非 isaac
16. FSM 停在 HOLD（`/arm_joint_cmd` 有数据但不动）
17. RViz Marker → target 断链
18. `task.info` 非法语法导致控制器加载失败
19. `/isaac_joint_states` 与 `/joint_states` 职责拆分，避免状态混发
20. `joint_state_fill.py` 补齐辅助关节，恢复完整 TF / RobotModel

---

## 十三、当前优先级

最高优先级不是直接训练，而是：

```text
1. 自动评测稳定
2. 数据筛选可信
3. 成功 / 失败标签准确
4. 轨迹能重复回放
5. 再进入训练
```

判断标准：

```text
没有自动评测 → 数据扩增可能只是污染数据
有自动评测 → Isaac Sim 可以变成稳定的数据工厂
```
