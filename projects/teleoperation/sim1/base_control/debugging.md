# SIM1 真实差速底盘构建、问题排查与经验总结

> 目标：记录从“新增底盘移动节点/话题”到“真实差速轮底盘可前进、后退、圆弧转弯”的完整排查过程。  
> 当前结论：ROS2、Action Graph、Articulation、wheel DOF、PhysX 接触、符号映射均已基本打通。原地转向不适合当前 2 驱动轮 + 4 caster 结构，最终采用圆弧 90°转弯。

---

## 1. 最终实现方案

当前最终采用真实差速轮控制，不使用 kinematic root movement：

```text
ROS2 /sim1/base_diff_cmd
  → Isaac Sim /World/ActionGraphs/Base_Drive_Graph
  → ROS2SubscribeTwist
  → ScriptNode
  → IsaacArticulationController.velocityCommand
  → wheel_L_joint / wheel_R_joint
```

控制语义：

```text
linear.x > 0  前进
linear.x < 0  后退
angular.z > 0 触发左转 90°圆弧
angular.z < 0 触发右转 90°圆弧
```

底盘结构：

```text
主动轮：
  wheel_L_joint
  wheel_R_joint

随动轮：
  caster_LF_joint
  caster_RF_joint
  caster_LB_joint
  caster_RB_joint
```

---

## 2. 关键参数记录

当前已验证可用参数：

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

当前经验：

```text
直行速度主要由 linear.x 控制。
圆弧转弯速度主要由 ARC_LINEAR_SPEED / ARC_RADIUS / MAX_WHEEL_SPEED 控制。
angular.z 在当前脚本里主要作为“左/右转触发信号”，不是精确角速度。
```

---

## 3. 问题 1：ROS2 topic 一开始不存在

### 现象

```bash
ros2 topic info /sim1/base_diff_cmd -v
```

输出：

```text
Unknown topic '/sim1/base_diff_cmd'
```

### 根因

`ROS2SubscribeTwist` 节点的 `topicName` 被设置成：

```text
sub_base_diff_cmd
```

这会导致 Isaac 实际订阅的是：

```text
/sub_base_diff_cmd
```

而不是：

```text
/sim1/base_diff_cmd
```

### 解决

在 Isaac Graph 中把：

```text
Base_Drive_Graph/sub_base_diff_cmd.inputs:topicName
```

改为：

```text
/sim1/base_diff_cmd
```

然后 Stop → Play。

### 验证

```bash
ros2 topic list | grep sim1
ros2 topic info /sim1/base_diff_cmd -v
```

期望：

```text
/sim1/base_diff_cmd
Type: geometry_msgs/msg/Twist
Subscription count: 1
```

### 经验

ROS2 Bridge 的 topicName 必须显式检查。不要只看节点名，也不要假设节点名等于 topic 名。

---

## 4. 问题 2：误判 Articulation 不存在

### 现象

执行：

```python
from omni.isaac.dynamic_control import _dynamic_control
dc = _dynamic_control.acquire_dynamic_control_interface()
print(dc.get_articulation("/World/Robot"))
```

输出：

```text
0
```

一开始误判为机器人没有 Articulation。

### 根因

当前 Stage 里：

```text
/World/Robot
```

只是外层 Xform。

真正 ArticulationRoot 在：

```text
/World/Robot/base_link
```

正确验证：

```python
print("Robot:", dc.get_articulation("/World/Robot"))
print("Base :", dc.get_articulation("/World/Robot/base_link"))
```

结果：

```text
Robot: 0
Base : 非零 handle
```

### 解决

Action Graph 或 dynamic_control 相关逻辑中，优先使用：

```text
/World/Robot/base_link
```

### 经验

`dc.get_articulation("/World/Robot") = 0` 不一定是错误。必须先确认 Stage 中 ArticulationRootAPI 具体挂在哪个 prim 上。

---

## 5. 问题 3：PhysicsScene 污染导致机器人飞

### 现象

Play → Stop → Play 后，机器人飞起或出现 PhysX error：

```text
PhysX error: Illegal BroadPhaseUpdateData
```

### 根因

场景里曾出现两个 PhysicsScene：

```text
/World/PhysicsScene
/physicsScene
```

其中错误 scene 可能包含：

```text
gravityDirection = (0,0,0)
gravityMagnitude = -inf
```

### 解决

清理错误 `/physicsScene`，只保留：

```text
/World/PhysicsScene
gravityDirection = (0,0,-1)
gravityMagnitude = 9.81
```

同时保存为干净 USD，重新打开场景，不要在飞过的污染状态继续调试。

### 经验

底盘调试前必须先保证 PhysicsScene 干净。否则所有控制、摩擦、Drive 调试都会被污染物理状态误导。

---

## 6. 问题 4：Drive Type 误解

### 现象

wheel joint UI 中 Drive Type 只有：

```text
force
acceleration
```

一开始容易误以为“没有 velocity drive”。

### 正确认识

Isaac/PhysX 中 Drive Type 是驱动力模型，不是“位置/速度模式”选择。

对 articulation velocity target，通常仍使用：

```text
Drive Type = force
Stiffness = 0
Damping > 0
Max Force > 0
Target Velocity = 0
```

推荐 wheel angular drive：

```text
Type = force
Stiffness = 0
Damping = 500
Max Force = 1000
Target Velocity = 0
Target Position = 0
```

### 经验

不能要求 UI 里找不存在的 “velocity” 类型。速度控制靠 `velocityCommand` + angular drive damping 生效。

---

## 7. 问题 5：wheel joint 有速度但底盘不动

### 现象

`/isaac_joint_states` 中：

```text
wheel_L_joint velocity ≠ 0
wheel_R_joint velocity ≠ 0
effort ≠ 0
```

但视觉上机器人不明显移动。

### 根因

控制链路已经通，但接触动力学未把轮速有效转换成底盘平动。可能原因包括：

```text
1. wheel / ground / caster 摩擦缺失或不合理
2. caster 大量滑移消耗能量
3. 速度太低，视觉上不明显
4. wheel drive damping / maxForce 不合适
```

### 解决过程

1. 先确认 ROS2 订阅正常。
2. 再看 `/isaac_joint_states` 中 wheel velocity 是否变化。
3. 发现 wheel velocity 有响应，说明不是 Graph/ROS2 问题。
4. 补充 wheel / caster / ground Physics Material。
5. 提高测试速度，例如 `linear.x=0.5`。
6. 最终确认机器人可以前进/后退。

### 经验

排查必须分层：

```text
ROS2 topic
→ Graph subscription
→ ScriptNode 输出
→ Articulation DOF
→ wheel velocity
→ base_link transform
→ ground contact
```

不要在 wheel 已经转动后继续怀疑 ROS2 topic。

---

## 8. 问题 6：PhysX Material API 走错

### 现象

尝试使用：

```python
UsdPhysics.Material
```

报错：

```text
AttributeError: module 'pxr.UsdPhysics' has no attribute 'Material'
```

### 根因

Isaac Sim 5.1 中不能按这个方式创建材质。

### 解决方向

使用：

```python
UsdShade.Material
UsdShade.Shader
UsdShade.MaterialBindingAPI
```

创建并绑定 PhysX material。也可以直接在 UI 中检查 material 是否绑定到具体 Collision shape。

### 经验

Isaac Sim 版本 API 变化较多，不能凭旧记忆写 USD/PhysX API。遇到 attribute 不存在时，优先用当前版本可用 schema 或本地文档确认。

---

## 9. 问题 7：前进命令变成后退

### 现象

命令：

```bash
ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.2}, angular: {z: 0.0}}"
```

机器人向后运动。

### 根因

wheel joint axis / USD 坐标系 / 机器人前向定义与脚本默认方向相反。

### 解决

在 ScriptNode 中修改：

```python
LEFT_SIGN = -1.0
RIGHT_SIGN = -1.0
```

修改后：

```text
linear.x > 0 → 机器人前进
linear.x < 0 → 机器人后退
```

### 经验

真实轮控必须预留 sign 参数：

```python
LEFT_SIGN = 1.0
RIGHT_SIGN = 1.0
```

并通过实测决定。不要假设 URDF/Isaac joint axis 与 ROS base_link 前向天然一致。

---

## 10. 问题 8：原地转向困难

### 现象

使用：

```bash
timeout 2.0 ros2 topic pub --rate 20 /sim1/base_diff_cmd geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 1.0}}"
```

转弯约 47°；提高到 `angular.z=2.0`，效果仍有限。

### 根因

当前底盘为：

```text
2 个主动差速轮 + 4 个 caster
```

原地转向时，caster 必须横向滑移并快速摆正，静摩擦、接触阻力和随动轮方向会导致：

```text
1. yaw 增益很低
2. 打滑明显
3. 车身抖动
4. 目标角附近反复修正
```

### 中间尝试

1. 读取 `/World/Robot/trunk_link` yaw，做 90°闭环原地转。
2. 后来改成读取 `/World/Robot/base_link` yaw。
3. 发现即使闭环，原地转仍然慢、抖、滑。

### 最终方案

放弃原地转向，改为：

```text
汽车式圆弧转弯：
vx = ARC_LINEAR_SPEED
wz = vx / ARC_RADIUS
```

并用 `base_link` 当前 yaw 闭环判断是否达到目标角。

### 经验

机器人底盘结构决定控制策略。对于 4 caster 结构，原地差速转向在仿真中容易不稳定；圆弧转向更符合当前物理结构。

---

## 11. 当前圆弧转弯逻辑

### 触发逻辑

```text
linear.x = 0 且 angular.z > 0
  → 锁定目标 yaw = 当前 yaw + 90°

linear.x = 0 且 angular.z < 0
  → 锁定目标 yaw = 当前 yaw - 90°
```

### 执行逻辑

```text
未到目标：
  vx = ARC_LINEAR_SPEED * scale
  wz = sign(error) * vx / ARC_RADIUS

到目标：
  vx = 0
  wz = 0
```

### 当前优点

```text
1. 不再依赖固定时间估算转角
2. 可以自动转到接近 ±90°
3. 比原地转向更容易让 caster 跟随
4. 适合阶段 B 的“左移”分解动作
```

### 当前缺点

```text
1. 圆弧半径较小且速度较高时仍有抖动
2. 实际路径不是严格几何圆弧，受接触和滑移影响
3. 角度闭环能保证朝向，但不能保证空间位置精确
```

---

## 12. 重要教训总结

### 12.1 不要只看 topic 是否有数据

`ros2 topic pub` 正常只说明 ROS2 发布成功。真正要判断闭环是否成功，需要看：

```text
/isaac_joint_states 中 wheel velocity
base_link transform
Isaac 视觉运动
```

### 12.2 不要把所有问题都归因到 OCS2

底盘控制是独立 Action Graph：

```text
/sim1/base_diff_cmd → Base_Drive_Graph
```

它不经过：

```text
/arm_joint_cmd
ocs2_arm_controller
arms_target_manager
```

底盘问题不要优先改 OCS2。

### 12.3 Articulation root 必须以 Stage 为准

外层 `/World/Robot` 不一定是 articulation root。当前有效 root 是：

```text
/World/Robot/base_link
```

### 12.4 真实差速不能纯左移

阶段 B 不能写成：

```text
base linear.y 左移
```

应写为：

```text
左转 90°
前进指定距离
右转 90°
```

也就是：

```text
B1_base_arc_left_90
B2_base_forward_to_output
B3_base_arc_right_90
```

### 12.5 物理仿真要接受“结构约束”

2 驱动轮 + 4 caster 的运动特性与全向底盘不同。即使控制公式正确，也会被接触、摩擦、caster 摆动影响。

### 12.6 保存可用状态比继续盲调更重要

当前已有可用参数，应保存：

```text
USD clean copy
ScriptNode working copy
Action Graph export
scene_structure export
```

否则下次调试很难还原。

---

## 13. 后续：轨迹记录方案

你最后提到“再考虑写一个轨迹记录脚本或手动操控与记录”。建议分两层做。

---

### 13.1 第一层：最简单的 ros2 bag 记录

这是最低风险方案，不改任何代码：

```bash
mkdir -p ~/ros2_log/base_bag

ros2 bag record \
  /sim1/base_diff_cmd \
  /isaac_joint_states \
  /joint_states \
  /clock \
  -o ~/ros2_log/base_bag/sim1_base_test_$(date +%Y%m%d_%H%M%S)
```

优点：

```text
1. 不改现有 SIM1 teach
2. 能完整保存命令和状态
3. 后续可离线分析 wheel velocity、joint position、命令时序
```

缺点：

```text
1. 不一定直接保存 base_link 世界坐标
2. 后处理相对麻烦
```

如果 `/tf` 可用，也可以加上：

```bash
/tf /tf_static
```

---

### 13.2 第二层：CSV 记录脚本

后续可以写一个 ROS2 Python 节点：

```text
sim1_base_trajectory_recorder.py
```

订阅：

```text
/sim1/base_diff_cmd
/isaac_joint_states
/joint_states
/tf 或 /odom
```

输出：

```text
~/teleoperation/sim1/trace_data/base/base_trace_YYYYmmdd_HHMMSS.csv
```

建议字段：

```text
wall_time
ros_time_sec
cmd_vx
cmd_wz
base_x
base_y
base_yaw_deg
wheel_L_pos
wheel_R_pos
wheel_L_vel
wheel_R_vel
caster_LF_vel
caster_RF_vel
caster_LB_vel
caster_RB_vel
motion_event
```

其中：

```text
motion_event:
  base_forward
  base_backward
  base_arc_left_90
  base_arc_right_90
  base_stop
```

如果能从 TF 读到：

```text
world/odom → base_link
```

则直接记录 base 位姿。如果 TF 不稳定，后续也可以在 Isaac ScriptNode 里额外发布：

```text
/sim1/base_pose
geometry_msgs/msg/PoseStamped
```

---

### 13.3 第三层：接入 SIM1 teach

等 CSV recorder 独立验证后，再把底盘字段接入原示教数据：

主 CSV 增加：

```text
base_cmd_vx
base_cmd_wz
base_x
base_y
base_yaw_deg
base_motion_event
```

diagnostics CSV 增加：

```text
base_motion_enabled
base_turn_active
base_turn_target_yaw_deg
base_turn_error_deg
base_blocked_multi_arrow_count
```

同时更新：

```text
GAME_GUIDE.md
TRACE_DATA_FORMAT.md
replay_ocs2_target_trace.py
```

### 13.4 推荐实施顺序

```text
1. 先用 ros2 bag record 保存可用实验
2. 写独立 base CSV recorder，不接入主 teach
3. 验证 CSV 中 base_x/base_y/base_yaw 是否可信
4. 再接入 keyboard_ocs2_gripper_teleop.py
5. 最后做 replay，同步回放底盘 + 双臂 + 夹爪
```

---

## 14. 当前建议的下一步

当前底盘已经可用，下一步不要继续大改物理结构。建议进入“数据化验证”：

```text
1. 用现有参数跑 3~5 次：
   前进 1m → 左转 90° → 前进 2m → 右转 90° → 前进 1m

2. 记录：
   每段实际耗时
   base_link 起终点
   yaw 误差
   是否明显抖动
   wheel_L/R 速度

3. 再决定：
   是写 ros2 bag + 后处理
   还是直接写 CSV recorder
```
