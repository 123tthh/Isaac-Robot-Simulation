# SIM1 真实差速底盘命令与使用手册

> 适用对象：Isaac Sim 5.1 中 `/World/Robot` 轮式双臂机器人  
> 当前底盘控制话题：`/sim1/base_diff_cmd`  
> 消息类型：`geometry_msgs/msg/Twist`  
> 当前控制方式：真实差速轮控制，主动控制 `wheel_L_joint` / `wheel_R_joint`，caster 作为随动轮  
> 当前推荐状态：直行、后退、圆弧左转 90°、圆弧右转 90°均可用

---

## 1. 当前控制链路

```text
ROS2 /sim1/base_diff_cmd
  → Isaac Sim /World/ActionGraphs/Base_Drive_Graph
  → ROS2 Subscribe Twist
  → ScriptNode 差速/圆弧转弯解算
  → Isaac Articulation Controller velocityCommand
  → wheel_L_joint / wheel_R_joint
  → PhysX 接触驱动底盘运动
```

当前控制**不经过 OCS2**，与双臂 `/arm_joint_cmd`、夹爪控制、相机发布并行运行。

---

## 2. 使用前检查

### 2.1 确认 Isaac Sim 已经 Play

先打开 USD 场景，运行 `Base_Drive_Graph` 对应脚本，确保 Isaac Sim 处于 Play 状态。

### 2.2 检查 ROS2 topic

```bash
ros2 topic list | grep sim1
ros2 topic info /sim1/base_diff_cmd -v
```

期望看到：

```text
/sim1/base_diff_cmd
Type: geometry_msgs/msg/Twist
Subscription count: 1
Node name: _World_ActionGraphs_Base_Drive_Graph_sub_base_diff_cmd
```

如果 `Unknown topic '/sim1/base_diff_cmd'`，优先检查 Isaac Graph 中：

```text
/World/ActionGraphs/Base_Drive_Graph/sub_base_diff_cmd.inputs:topicName
```

必须是：

```text
/sim1/base_diff_cmd
```

注意必须带 `/`。

---

## 3. 清理残留发布器

在测试前建议先杀掉之前未停止的 `ros2 topic pub --rate`：

```bash
pkill -f "ros2 topic pub.*base_diff_cmd" || true
```

如果怀疑仍有多个 publisher：

```bash
ros2 topic info /sim1/base_diff_cmd -v | grep -E "Publisher count|Node name"
```

正常手动测试时，`Publisher count` 应只包含当前正在运行的临时发布器。

---

## 4. 基本命令

### 4.1 前进

```bash
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

说明：

```text
linear.x > 0 机器人前进
linear.x = 0.5 表示期望底盘前进速度约 0.5 m/s
```

实际速度会受到轮地接触、caster、摩擦、Drive 参数、PhysX 时间步影响。

### 4.2 后退

```bash
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: -0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

说明：

```text
linear.x < 0 机器人后退
```

### 4.3 停止

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

如果是 `--rate` 持续发布命令，按 `Ctrl+C` 后建议再发一次停止命令。

---

## 5. 圆弧转弯命令

当前脚本已经不建议原地转向。原因是该底盘为：

```text
2 个主动差速轮 + 4 个 caster 随动轮
```

原地转向时 caster 需要大幅横向滑移，阻力和抖动明显。因此当前采用：

```text
angular.z > 0 → 触发一次左转 90°圆弧运动
angular.z < 0 → 触发一次右转 90°圆弧运动
```

### 5.1 左转 90°圆弧

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

语义：

```text
不是持续 angular.z = 1.0 rad/s
而是触发一次左转 90°闭环圆弧动作
```

脚本会读取：

```text
/World/Robot/base_link
```

的 yaw 角，锁定目标角：

```text
当前 yaw + 90°
```

未到目标角会继续转，到达后自动停车。

### 5.2 重新解锁下一次转向

每次圆弧转弯完成后，建议发一次零命令，重新解锁下一次转向：

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 5.3 右转 90°圆弧

```bash
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
```

脚本会锁定目标角：

```text
当前 yaw - 90°
```

---

## 6. 当前脚本关键参数

当前你已经验证可用的参数组如下。

```python
WHEEL_RADIUS = 0.10
WHEEL_SEPARATION = 0.55
MAX_WHEEL_SPEED = 25

LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0

LINEAR_GAIN = 1.0
RAW_TURN_GAIN = 1.0

ENABLE_ARC_90_TURN = True
YAW_PRIM_PATH = "/World/Robot/base_link"
ARC_TARGET_DEG = 90.0
ARC_RADIUS = 0.1
ARC_LINEAR_SPEED = 0.5
ARC_DONE_EPS_DEG = 2.0
ARC_SLOWDOWN_DEG = 25.0
ARC_MIN_SPEED_SCALE = 0.45
ZERO_CMD_CANCELS_ARC = False
```

### 6.1 参数含义

| 参数 | 当前值 | 含义 |
|---|---:|---|
| `WHEEL_RADIUS` | `0.10` | 主动轮半径，影响 m/s 到 rad/s 的换算 |
| `WHEEL_SEPARATION` | `0.55` | 左右主动轮中心距，影响转向差速 |
| `MAX_WHEEL_SPEED` | `25` | 左右轮角速度上限 |
| `LEFT_SIGN` | `-1.0` | 左轮方向修正 |
| `RIGHT_SIGN` | `-1.0` | 右轮方向修正 |
| `LINEAR_GAIN` | `1.0` | 直行速度增益 |
| `ENABLE_ARC_90_TURN` | `True` | 启用 angular.z 触发 90°圆弧转向 |
| `YAW_PRIM_PATH` | `/World/Robot/base_link` | 读取底盘 yaw 的 prim |
| `ARC_TARGET_DEG` | `90.0` | 每次触发转向的角度 |
| `ARC_RADIUS` | `0.1` | 圆弧转弯半径 |
| `ARC_LINEAR_SPEED` | `0.5` | 圆弧转弯时的前进速度 |
| `ARC_DONE_EPS_DEG` | `2.0` | 到达目标角的误差阈值 |
| `ARC_SLOWDOWN_DEG` | `25.0` | 接近目标多少度开始减速 |
| `ARC_MIN_SPEED_SCALE` | `0.45` | 减速区最低速度比例 |
| `ZERO_CMD_CANCELS_ARC` | `False` | 外部发零命令时是否取消正在执行的圆弧转弯 |

### 6.2 控制速度的方法

直行/后退速度主要由 ROS 命令中的 `linear.x` 控制：

```bash
# 慢速前进
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 快速前进
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

圆弧转弯速度主要由脚本参数控制：

```python
ARC_LINEAR_SPEED = 0.5
ARC_RADIUS = 0.1
MAX_WHEEL_SPEED = 25
```

在当前闭环圆弧脚本里，`angular.z` 的数值大小主要用于判断方向。推荐继续使用：

```text
左转：angular.z = +1.0
右转：angular.z = -1.0
```

不要把 `angular.z` 当作精确角速度。

---

## 7. 推荐完整路径测试

任务：

```text
前进 1m
左转 90°
前进 2m
右转 90°
前进 1m
```

### 7.1 手动时间估算版

假设直行速度使用：

```text
linear.x = 0.5 m/s
```

则：

```text
前进 1m ≈ 2s
前进 2m ≈ 4s
```

命令：

```bash
pkill -f "ros2 topic pub.*base_diff_cmd" || true

# 前进 1m
timeout 2.0 ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 左转 90°
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"

sleep 3

ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 前进 2m
timeout 4.0 ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 右转 90°
ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"

sleep 3

ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 前进 1m
timeout 2.0 ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

ros2 topic pub --once /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 7.2 更可靠的闭环版

后续建议增加“位移闭环”：

```text
读取 /World/Robot/base_link 世界坐标
记录起点 xy
持续前进直到 xy 距离达到 1m 或 2m
```

这样可以避免轮子打滑导致“时间估算距离”不准。

---

## 8. 当前底盘操作建议

### 8.1 放置阶段避让

右臂放完托盘后：

```text
1. 底盘后退 0.10~0.20m
2. 双臂进行托盘交接
3. 底盘前进回到放置位
```

建议先用：

```bash
# 后退约 0.2m，线速度 0.2m/s，持续 1s
timeout 1.0 ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 8.2 阶段 B：到输出传送带

真实差速底盘不能纯左移。建议改写为：

```text
B1_base_arc_left_90
B2_base_forward_to_output
B3_base_arc_right_90
```

对应操作：

```text
左转 90°
前进指定距离
右转 90°
```

---

## 9. 常见问题速查

### 9.1 `ros2 topic info` Unknown topic

检查 Isaac Graph 里 `topicName` 是否是：

```text
/sim1/base_diff_cmd
```

不是 `sub_base_diff_cmd`。

### 9.2 前进命令变成后退

修改 ScriptNode：

```python
LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0
```

当前你已经确认该组参数可用。

### 9.3 轮子转但底盘不动

按顺序检查：

```text
1. /isaac_joint_states 里 wheel_L_joint / wheel_R_joint velocity 是否变化
2. wheel joint Drive damping 是否非零
3. wheel / ground / caster 是否有 Physics Material
4. base_link 是否为有效 ArticulationRoot
5. 是否仍存在错误 PhysicsScene，例如 gravityMagnitude = -inf
```

### 9.4 原地转向很困难

这是当前结构的物理特性：

```text
2 个主动差速轮 + 4 个 caster
```

原地转向需要 caster 大范围横向滑移，容易抖动。当前推荐用圆弧转向，不再追求原地转向。

---

## 10. 最小保存建议

当前这套参数已经可用，建议保存：

```text
68wheel_base_drive_working.usd
```

并把 ScriptNode 代码另存为：

```text
~/teleoperation/sim1/isaac_script_editor_diagnostics/script_diff_drive_arc90_working.py
```

同时导出当前 Stage / Action Graph，便于后续对话恢复：

```text
~/ros2_log/scripts/Graph/
```
