# Claude Code 任务：Isaac Sim Action Graph 节点配置（支持感知+定位+TF）

## 背景
参考 ~/ros2_ws/CLAUDE.md。

上一步已决定删除 Isaac Sim 的 publish_tf 节点（解决 TF 双源冲突）。
但后续强化学习需要：
1. 相机检测托盘位姿（需要完整 TF 链：camera → base_link → world）
2. 底盘移动时的全局定位（需要 odom → base_link 动态 TF）

## 最终架构设计

### TF 发布职责划分

| 变换链 | 谁发布 | 方式 |
|--------|--------|------|
| world → odom | odom_to_tf.py (ROS2) | 静态 identity |
| odom → base_link | odom_to_tf.py (ROS2) | 动态，从 /odom 话题读取 |
| base_link → 所有子 frame | planning_robot_state_publisher (ROS2) | 从 /joint_states + URDF FK |
| Robot → base_link | static_transform_publisher (ROS2) | 静态 identity (桥接用) |

### Isaac Sim 发布的数据（不含 TF）

| 话题 | 节点 | Action Graph |
|------|------|-------------|
| /joint_states | ROS2 Publish Joint State | State_Telemetry_Graph |
| /clock | ROS2 Publish Clock | State_Telemetry_Graph |
| /odom | ROS2 Publish Odometry | State_Telemetry_Graph ← 新增 |
| /head_cam/* | ROS2 Camera Helper ×2 | Camera_Publish_Graph |
| /left_cam/* | ROS2 Camera Helper ×2 | Camera_Publish_Graph |
| /right_cam/* | ROS2 Camera Helper ×2 | Camera_Publish_Graph |

## 步骤1：Isaac Sim — 确认/添加 Odometry 发布

### 检查 /odom 是否已经在发布

在终端运行:
```bash
ros2 topic hz /odom
ros2 topic echo /odom --once
```

如果已经有数据（之前话题列表里确实有 /odom），说明 Isaac Sim 已经在
通过某种方式发布里程计（可能是导航栈或底盘控制器自带的）。
这种情况下不需要额外添加节点。

### 如果 /odom 没有数据

在 Isaac Sim 的 State_Telemetry_Graph 中添加:

1. 搜索并拖入: `isaacsim.ros2.bridge.ROS2PublishOdometry`
2. 连线:
   - on_playback_tick.Tick → ROS2PublishOdometry.ExecIn
   - ros2_context.Context → ROS2PublishOdometry.Context
   - read_sim_time.SimulationTime → ROS2PublishOdometry.TimeStamp
3. 设置属性:
   - chassisPrim: `/World/Robot` (或 `/World/Robot/base_link`)
   - topicName: `/odom`
   - frameId: `odom`
   - childFrameId: `base_link`
   - nodeNamespace: 留空

注意：ROS2PublishOdometry 节点有一个 `publishTF` 选项，
**必须设为 false**！否则它会再次发布 odom→base_link 的 TF，
和 odom_to_tf.py 冲突。我们只用它发布 /odom 话题数据。

## 步骤2：Isaac Sim — 删除 publish_tf（确认已完成）

State_Telemetry_Graph 最终应只包含：
```
on_playback_tick
ros2_context
read_sim_time
ros2_publish_joint_state  ← 保留（发布 /joint_states）
ros2_publish_clock        ← 保留（发布 /clock）
ros2_publish_odometry     ← 新增或已有（发布 /odom，publishTF=false）
```

不要有 publish_tf（ROS2PublishTransformTree）节点。

## 步骤3：ROS2 launch 文件 — 最终版

修改 ~/ros2_ws/src/r1_description/launch/ocs2_isaac.launch.py

在 launch_setup 函数中，节点定义部分应包含：

### 3.1 joint_state_publisher（合并关节状态）

```python
    # ------------------------------------------------------------------
    # 合并 Isaac Sim 的 /joint_states 与 URDF 中未被覆盖的关节默认值
    # 确保 planning_robot_state_publisher 能算完整 TF 树
    # ------------------------------------------------------------------
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="auxiliary_joint_state_publisher",
        parameters=[
            {"use_sim_time": True},
            {"source_list": ["/joint_states"]},
            {"rate": 60},
        ],
        remappings=[
            # 输出到不同话题，避免和 Isaac Sim 的 /joint_states 冲突
            ("/joint_states", "/merged_joint_states"),
        ],
        output="log",
    )
```

等等——这里有个问题。joint_state_publisher 的 source_list 是它订阅的话题。
如果我们 remap 它的输出为 /merged_joint_states，
那 planning_robot_state_publisher 也需要订阅 /merged_joint_states。

但更简单的方式是：不用 joint_state_publisher，
而是让 planning_robot_state_publisher 直接订阅 /joint_states，
然后它会自动忽略 URDF 中不存在的关节，
缺失的关节会使用其 URDF 默认值（通常是 0）。

实际上 robot_state_publisher 已经能处理这种情况——
它会对 /joint_states 中存在的关节更新位姿，
对不存在的关节保持上次值或默认值。

所以问题只出在 URDF 中的关节名和 /joint_states 中的关节名不匹配。

让我们先检查不匹配的关节是哪些。

### 3.2 诊断命令（在终端执行）

```bash
# 列出 /joint_states 中的关节名
ros2 topic echo /joint_states --once --field name

# 列出 URDF 中所有非 fixed 关节
python3 -c "
import xml.etree.ElementTree as ET
tree = ET.parse('$(ros2 pkg prefix r1_description)/share/r1_description/urdf/r1_fixed.urdf')
for j in tree.getroot().findall('joint'):
    if j.get('type') != 'fixed':
        print(f\"{j.get('name'):30s} type={j.get('type')}\")
"
```

如果某个关节在 URDF 中是 non-fixed 但 /joint_states 中没有，
那 robot_state_publisher 就无法计算该关节子树的 TF。

最可能的断点：
- head_camera_joint (continuous) — /joint_states 中的名称可能不匹配
- caster_*_joint — 可能也不匹配

修复方式：在 URDF 中把这些不需要动态控制的关节改为 fixed 类型。

### 3.3 修改 r1_fixed.urdf（如果诊断确认名称不匹配）

```bash
# 在 r1_fixed.urdf 中将 caster 和 head_camera 关节改为 fixed
cd ~/ros2_ws/src/r1_description/urdf/

# head_camera_joint: continuous → fixed
sed -i 's/joint name="head_camera_joint" type="continuous"/joint name="head_camera_joint" type="fixed"/' r1_fixed.urdf

# caster joints: continuous → fixed (如果确认不需要动态旋转)
for j in caster_LB_joint caster_LF_joint caster_RB_joint caster_RF_joint; do
    sed -i "s/joint name=\"${j}\" type=\"continuous\"/joint name=\"${j}\" type=\"fixed\"/" r1_fixed.urdf
done
```

注意：改为 fixed 后 robot_state_publisher 会把它们当作静态变换发布到
/tf_static，不再需要 /joint_states 中的数据。
缺点是 RViz 中万向轮和头部相机不会旋转，但这不影响 OCS2 控制。

如果确实需要 caster 轮可视化旋转（通常不需要），可以保留 continuous 类型
并添加 joint_state_publisher 来补充零位。

### 3.4 odom_to_tf 节点（已有，确认配置正确）

确保 odom_to_tf.py 发布:
- world → odom (静态 identity)
- odom → base_link (动态，从 /odom 话题)

### 3.5 最终 nodes 列表

```python
    nodes = [
        planning_robot_state_publisher,  # FK: /joint_states → TF (base_link 子树)
        odom_to_tf_node,                 # /odom → TF (world→odom→base_link)
        static_tf_robot,                 # 静态: Robot → base_link (Isaac Sim 桥接)
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[ocs2_arm_controller_spawner]),
    ]
```

## 步骤4：感知管线（后续任务，当前不实现）

托盘位姿检测的数据流：

```
Isaac Sim Camera_Publish_Graph
  ├── /head_cam/color/image_raw   ─┐
  ├── /head_cam/depth/image_rect_raw ─┤
  ├── /left_cam/color/image_raw   ─┤  → 感知节点 (Python/C++)
  ├── /left_cam/depth/image_rect_raw ─┤     │
  ├── /right_cam/color/image_raw  ─┤     │ 检测托盘 + 估计 6D 位姿
  └── /right_cam/depth/image_rect_raw ─┘     │ (在相机坐标系下)
                                              ↓
                                    tf2_ros.TransformListener
                                    lookup_transform("world", "camera_H_color_optical_frame")
                                              ↓
                                    托盘在 world 坐标系下的位姿
                                              ↓
                                    发布到 /tray_pose (geometry_msgs/PoseStamped)
                                              ↓
                                    OCS2 TargetTrajectories → MPC 规划
```

这条链的前提就是 TF 树完整（world → ... → camera_H_color_optical_frame）。
修复完 TF 后这条链自然就通了。

感知算法的选择（后续）：
- 简单场景：ArUco 标记 + OpenCV solvePnP
- 复杂场景：FoundationPose / MegaPose (基于 RGB-D 的 6D 位姿估计)
- RL 训练场景：Isaac Sim 内置的 ground truth pose（直接读 USD prim transform）

## 编译验证

```bash
cd ~/ros2_ws
colcon build --packages-select r1_description
source install/setup.bash

# Isaac Sim Play（确认已删除 publish_tf，已有/添加 publish_odometry）
ros2 launch r1_description ocs2_isaac.launch.py

# 验证
ros2 topic info /tf                    # 应只有 1-2 个 publisher
ros2 run tf2_ros tf2_echo world base_link  # 应有稳定输出，无 jump back 警告
ros2 run tf2_ros tf2_echo base_link camera_H_color_optical_frame  # 应有输出
```
