# SIM1 底盘轨迹记录接手说明

> 生成时间：2026-06-15 08:04:55  
> 目的：用于下一段对话 / Codex 任务，让接手者快速理解 SIM1 真实差速底盘控制已经打通后的轨迹记录背景、接口、字段设计、最小实现路径与后续接入 SIM1 teach 主链路的计划。

---

## 1. 当前背景

SIM1 工程已经完成 Isaac Sim 5.1 中 R1 轮式双臂机器人的真实差速底盘控制链路。

当前底盘控制不再使用 kinematic root movement，而是通过 ROS2 Twist 指令控制真实轮子关节：

```text
ROS2 /sim1/base_diff_cmd
  → Isaac Sim Base_Drive_Graph
  → ROS2SubscribeTwist
  → ScriptNode 差速/圆弧转弯解算
  → IsaacArticulationController.velocityCommand
  → wheel_L_joint / wheel_R_joint
  → PhysX 物理运动
```

当前已经确认：

```text
/sim1/base_diff_cmd topic 存在
Subscription count = 1
机器人可以前进、后退、圆弧左转、圆弧右转
wheel_L_joint / wheel_R_joint 在 /isaac_joint_states 中有速度反馈
base_link / trunk_link 的 yaw 可用于转向角闭环
```

---

## 2. 当前底盘控制接口

### 2.1 控制 topic

```text
/sim1/base_diff_cmd
```

消息类型：

```text
geometry_msgs/msg/Twist
```

当前只使用：

```text
linear.x   前进/后退速度，单位 m/s
angular.z  触发左/右转弯，单位 rad/s 或作为方向触发量
```

忽略：

```text
linear.y
linear.z
angular.x
angular.y
```

### 2.2 当前命令语义

```text
linear.x > 0  → 前进
linear.x < 0  → 后退
linear.x = 0, angular.z > 0  → 触发左转圆弧 90°
linear.x = 0, angular.z < 0  → 触发右转圆弧 90°
linear.x = 0, angular.z = 0  → 停止 / 解锁下一次转向
```

注意：现在不建议使用原地自转。由于 R1 底盘是：

```text
2 个主动轮 + 4 个 caster
```

原地转向时 caster 横向滑移和摆动非常明显，容易抖动；当前采用“汽车式圆弧转弯”更稳定。

---

## 3. 当前可用测试命令

### 3.1 检查 topic

```bash
ros2 topic info /sim1/base_diff_cmd -v
```

期望：

```text
Type: geometry_msgs/msg/Twist
Subscription count: 1
Node name: _World_ActionGraphs_Base_Drive_Graph_sub_base_diff_cmd
```

### 3.2 停止所有残留 publisher

```bash
pkill -f "ros2 topic pub.*base_diff_cmd" || true
```

### 3.3 前进

```bash
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.4 后退

```bash
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.5 停止

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.6 左转 90°圆弧

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

### 3.7 解锁下一次转向

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.8 右转 90°圆弧

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

---

## 4. 当前 ScriptNode 关键参数

以下参数来自当前调通版本，应先保留，不建议立刻大改：

```python
WHEEL_RADIUS = 0.10
WHEEL_SEPARATION = 0.55

LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0

YAW_PRIM_PATH = "/World/Robot/base_link"

ENABLE_ARC_90_TURN = True
ARC_TARGET_DEG = 90.0

# 用户当前调通版本倾向：
MAX_WHEEL_SPEED = 25
ARC_RADIUS = 0.1
ARC_LINEAR_SPEED = 0.5
```

说明：

1. `LEFT_SIGN = RIGHT_SIGN = -1.0` 是经过实测确认的方向修正。没有这两个负号时，`linear.x > 0` 会后退。
2. `YAW_PRIM_PATH` 从 `/World/Robot/trunk_link` 改成 `/World/Robot/base_link` 后，base_link 的 Z 角更适合作为底盘 yaw 反馈。
3. 原地转向困难，因此最终改成圆弧转弯。
4. 当前参数兼顾速度和转弯位置，但转弯过程中有一定抖动；用户表示当前参数“可行”，后续应先记录和集成，不要继续强行优化物理参数。

---

## 5. 需要记录的数据

底盘轨迹记录至少应记录三类数据：

1. 用户/控制器发出的底盘命令；
2. Isaac Sim 中机器人真实底盘状态；
3. 轮子关节状态，用于判断是否打滑或转向效率不足。

---

## 6. 推荐记录 topic

### 6.1 必录 topic

```text
/sim1/base_diff_cmd
/isaac_joint_states
/joint_states
/clock
```

含义：

| Topic | 类型 | 作用 |
|---|---|---|
| `/sim1/base_diff_cmd` | `geometry_msgs/msg/Twist` | 记录底盘输入命令 |
| `/isaac_joint_states` | `sensor_msgs/msg/JointState` | Isaac 原始关节状态，包含 wheel/caster 速度和 effort |
| `/joint_states` | `sensor_msgs/msg/JointState` | ROS2 系统统一状态，便于和现有 SIM1 数据链路对齐 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真时间戳 |

### 6.2 建议录制 topic

```text
/tf
/tf_static
```

如果 ROS2 侧已有 `robot_state_publisher` 和完整 TF，可以通过 TF 得到 `base_link` 的位姿；否则应考虑在 Isaac Sim 或 ROS2 侧直接发布一个 `/sim1/base_pose` 或 `/odom`。

### 6.3 如果已有 odom

如果后续已有 `/odom`，也应记录：

```text
/odom
```

类型：

```text
nav_msgs/msg/Odometry
```

---

## 7. 推荐 ros2 bag 记录方案

### 7.1 最小 bag 记录

```bash
mkdir -p ~/teleoperation/sim1/trace_data/base_bag

ros2 bag record \
  /sim1/base_diff_cmd \
  /isaac_joint_states \
  /joint_states \
  /clock \
  -o ~/teleoperation/sim1/trace_data/base_bag/base_drive_$(date +%Y%m%d_%H%M%S)
```

### 7.2 带 TF 的 bag 记录

```bash
mkdir -p ~/teleoperation/sim1/trace_data/base_bag

ros2 bag record \
  /sim1/base_diff_cmd \
  /isaac_joint_states \
  /joint_states \
  /clock \
  /tf \
  /tf_static \
  -o ~/teleoperation/sim1/trace_data/base_bag/base_drive_$(date +%Y%m%d_%H%M%S)
```

### 7.3 如果有 odom

```bash
ros2 bag record \
  /sim1/base_diff_cmd \
  /isaac_joint_states \
  /joint_states \
  /odom \
  /clock \
  /tf \
  /tf_static \
  -o ~/teleoperation/sim1/trace_data/base_bag/base_drive_$(date +%Y%m%d_%H%M%S)
```

---

## 8. CSV recorder 设计

后续建议新增独立脚本：

```text
~/teleoperation/sim1/base_drive_recorder.py
```

输出目录：

```text
~/teleoperation/sim1/trace_data/base_raw/
```

输出文件：

```text
base_drive_trace_YYYYmmdd_HHMMSS.csv
base_drive_trace_latest.csv
```

---

## 9. CSV 字段建议

### 9.1 时间字段

```text
wall_time
ros_time_sec
sim_time_sec
```

### 9.2 命令字段

```text
base_cmd_vx
base_cmd_wz
base_cmd_mode
base_motion_event
```

含义：

| 字段 | 含义 |
|---|---|
| `base_cmd_vx` | `/sim1/base_diff_cmd.linear.x` |
| `base_cmd_wz` | `/sim1/base_diff_cmd.angular.z` |
| `base_cmd_mode` | `forward/backward/left_arc_90/right_arc_90/stop/manual` |
| `base_motion_event` | 本帧事件，例如 `cmd_forward`、`cmd_left_arc`、`cmd_stop` |

### 9.3 关节字段

```text
wheel_L_position
wheel_R_position
wheel_L_velocity
wheel_R_velocity
wheel_L_effort
wheel_R_effort

caster_LF_position
caster_RF_position
caster_LB_position
caster_RB_position
caster_LF_velocity
caster_RF_velocity
caster_LB_velocity
caster_RB_velocity
```

### 9.4 位姿字段

如果能从 TF 或 `/odom` 得到底盘位姿，记录：

```text
base_x
base_y
base_z
base_roll
base_pitch
base_yaw_deg
base_yaw_rad
```

如果暂时没有 `/odom` 或 TF 解析不稳定，可以先只记录 joint state 和 command，后面再补位姿。

### 9.5 转弯闭环诊断字段

如果能把 ScriptNode 内部状态通过 ROS2 topic 发布或在 recorder 里估计，可记录：

```text
arc_active
arc_target_yaw_deg
arc_current_yaw_deg
arc_yaw_error_deg
arc_radius
arc_linear_speed
arc_done
```

如果暂时不能直接从 ScriptNode 拿内部状态，可先在 CSV 中留空。

---

## 10. 手动控制 + 记录的推荐流程

### 10.1 启动 Isaac Sim

```text
打开 68wheel_clean.usd / 当前调通 USD
确认 Isaac Sim 已 Play
确认 Base_Drive_Graph 已加载当前圆弧转弯 ScriptNode
```

### 10.2 启动 ROS2 环境

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

或容器环境下：

```bash
source /opt/ros/humble/setup.bash
source /workspace/projects/ros2_ws/install/setup.bash
```

### 10.3 检查底盘 topic

```bash
ros2 topic info /sim1/base_diff_cmd -v
```

### 10.4 启动记录

第一阶段建议先用 ros2 bag：

```bash
ros2 bag record \
  /sim1/base_diff_cmd \
  /isaac_joint_states \
  /joint_states \
  /clock \
  -o ~/teleoperation/sim1/trace_data/base_bag/base_drive_$(date +%Y%m%d_%H%M%S)
```

### 10.5 手动执行动作

例如：

```bash
# 前进
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 停止
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转圆弧 90°
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

# 解锁
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 11. 后续接入 SIM1 teach 主链路的建议

SIM1 当前已有人工示教数据目录：

```text
trace_data/raw/
trace_data/cleaned/
trace_data/lerobot_v30/
```

后续可将底盘控制接入现有 `keyboard_ocs2_gripper_teleop.py`。

### 11.1 建议新增主 CSV 字段

在原始示教 CSV 中新增：

```text
base_cmd_vx
base_cmd_wz
base_cmd_mode
base_left_wheel_cmd
base_right_wheel_cmd
base_x
base_y
base_yaw_deg
base_motion_event
```

### 11.2 建议新增 diagnostics 字段

在诊断 CSV 中新增：

```text
base_motion_enabled
base_blocked_multi_arrow_count
base_arc_active
base_arc_target_yaw_deg
base_arc_error_deg
base_wheel_slip_hint
```

### 11.3 pygame 键位建议

```text
↑       前进
↓       后退
←       左转圆弧 90°
→       右转圆弧 90°
方向键释放  停止 / 解锁
```

也可以设计更精细的模式：

```text
Shift + ← / →   小角度转弯
Ctrl  + ← / →   90°转弯
```

---

## 12. 独立 CSV recorder 最小伪代码

```python
class BaseDriveRecorder(Node):
    def __init__(self):
        self.cmd = None
        self.joint_state = None
        self.isaac_joint_state = None

        self.create_subscription(Twist, "/sim1/base_diff_cmd", self.on_cmd, 10)
        self.create_subscription(JointState, "/isaac_joint_states", self.on_isaac_js, 10)
        self.create_subscription(JointState, "/joint_states", self.on_js, 10)

        self.timer = self.create_timer(1.0 / 50.0, self.write_row)

    def write_row(self):
        # 1. 取最近一次 cmd
        # 2. 从 isaac_joint_states 中抽 wheel/caster
        # 3. 如果有 /odom 或 TF，抽 base pose
        # 4. 写 CSV 行
        pass
```

---

## 13. 下一段对话可直接给 Codex 的任务

```text
当前 SIM1 工程已经完成真实差速底盘控制，控制 topic 为 /sim1/base_diff_cmd，消息类型 geometry_msgs/msg/Twist，只使用 linear.x 和 angular.z。

当前 Base_Drive_Graph 已经可以控制 wheel_L_joint / wheel_R_joint。前进、后退、左/右圆弧 90°转弯均已可用。当前约定：
- linear.x > 0：前进
- linear.x < 0：后退
- linear.x = 0 且 angular.z > 0：左转圆弧 90°
- linear.x = 0 且 angular.z < 0：右转圆弧 90°
- linear.x = 0 且 angular.z = 0：停止/解锁

请在 ~/teleoperation/sim1 中新增底盘轨迹记录能力，先不要改 OCS2 和夹爪逻辑。

任务：
1. 新增 scripts/base_drive_recorder.py 或 base_drive_recorder.py。
2. 订阅：
   - /sim1/base_diff_cmd
   - /isaac_joint_states
   - /joint_states
   - /clock
   可选：
   - /odom
   - /tf
3. 以 50Hz 写 CSV：
   - trace_data/base_raw/base_drive_trace_YYYYmmdd_HHMMSS.csv
   - trace_data/base_raw/base_drive_trace_latest.csv 软链接
4. CSV 字段至少包含：
   - wall_time
   - ros_time_sec
   - sim_time_sec
   - base_cmd_vx
   - base_cmd_wz
   - base_cmd_mode
   - wheel_L_position
   - wheel_R_position
   - wheel_L_velocity
   - wheel_R_velocity
   - wheel_L_effort
   - wheel_R_effort
   - caster_LF_position
   - caster_RF_position
   - caster_LB_position
   - caster_RB_position
   - caster_LF_velocity
   - caster_RF_velocity
   - caster_LB_velocity
   - caster_RB_velocity
   如果能从 /odom 或 tf 读到底盘位姿，也记录 base_x/base_y/base_z/base_yaw_deg。
5. 增加一个最小 CLI：
   python3 base_drive_recorder.py --rate 50 --out-dir trace_data/base_raw
6. 增加 ros2 bag 记录命令说明。
7. 更新 GAME_GUIDE.md 和 TRACE_DATA_FORMAT.md，只添加底盘记录章节，不要破坏原有示教格式。
8. 完成后给出测试命令和一个 30 秒手动记录流程。
```

---

## 14. 当前不要做的事

1. 不要再修改 OCS2 `/arm_joint_cmd` 链路。
2. 不要修改夹爪话题和夹爪状态机。
3. 不要把底盘 topic 改成 `/cmd_vel`，继续使用 `/sim1/base_diff_cmd`。
4. 不要强行恢复原地转弯，当前结构下原地转弯物理效果差。
5. 不要依赖 `linear.y`，真实差速底盘不支持纯横移。
6. 不要让 Isaac 同时发布多个冲突的 `/joint_states`。当前应以 `/isaac_joint_states` 为原始状态源，`/joint_states` 为 ROS2 统一状态源。

---

## 15. 建议验收标准

底盘记录功能初版完成后，至少通过：

```text
1. 执行 ros2 bag record 能记录 /sim1/base_diff_cmd 和 /isaac_joint_states。
2. 独立 CSV recorder 能 50Hz 输出 CSV。
3. CSV 中能看到 base_cmd_vx/base_cmd_wz 随手动命令变化。
4. CSV 中能看到 wheel_L/R velocity 随命令变化。
5. 左转/右转时 wheel_L/R velocity 有明显差异。
6. 停止命令后 wheel_L/R velocity 逐渐归零。
7. 如果有 base pose，base_yaw_deg 能反映圆弧转弯前后的角度变化。
8. latest.csv 软链接能指向最近一次记录文件。
```

---

## 16. 结论

底盘轨迹记录的第一阶段不需要改控制链路。

推荐顺序：

```text
第一步：ros2 bag record 保存原始 topic
第二步：独立 base_drive_recorder.py 输出 CSV
第三步：把底盘字段接入 SIM1 teach 主 CSV
第四步：把底盘动作和双臂/夹爪示教整合成完整任务循环数据
```

这样风险最低，也便于后续转为 LeRobot / RL / VLA 所需的数据格式。
