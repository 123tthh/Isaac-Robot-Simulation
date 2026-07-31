# Claude Code 任务：ocs2_isaac.launch.py 最终修改 + URDF head_camera 修复

## 背景
参考 ~/ros2_ws/MD/CLAUDE.md。

Isaac Sim 侧已完成：
- ✅ publish_tf 节点已删除（TF 双源冲突已解决）
- ✅ /odom 话题已正常发布（frame_id=odom, child_frame_id=base_link）
- /odom 数据: position(x=0.175, y=3.18, z=-0.094), yaw≈180°（机器人朝向 -Y 方向）

## 任务1：修复 r1_fixed.urdf — head_camera_joint → fixed(已经完成)

原因：head_camera_joint 是 continuous 类型，但 Isaac Sim 中没有对应的驱动器，
/joint_states 中可能没有该关节状态，导致 robot_state_publisher 无法发布
head_camera_link 及其子树（camera_H_*）的 TF。
caster 关节保持 continuous 不变（有真实物理旋转）。

```bash
cd ~/ros2_ws/src/r1_description/robot/urdf/

# 只把 head_camera_joint 改为 fixed
sed -i 's/joint name="head_camera_joint" type="continuous"/joint name="head_camera_joint" type="fixed"/' r1_fixed.urdf

# 验证修改成功
grep 'head_camera_joint' r1_fixed.urdf
# 应输出: <joint name="head_camera_joint" type="fixed">
```

## 任务2：修改 ocs2_isaac.launch.py — 添加缺失节点

修改文件: ~/ros2_ws/src/r1_description/launch/ocs2_isaac.launch.py

### 2.1 添加 joint_state_publisher 节点（在 planning_robot_state_publisher 之前）

用途：把 Isaac Sim 的 /joint_states（27个关节）和 URDF 中剩余
non-fixed 关节（caster_*、head_camera 改 fixed 后就不需要了）合并。
主要作用是确保 caster_* 关节状态被 robot_state_publisher 接收到。

在 launch_setup 函数中，planning_robot_state_publisher 节点定义之前添加：

```python
    # ------------------------------------------------------------------
    # 0. joint_state_publisher
    #    合并 Isaac Sim /joint_states 与 URDF 未覆盖关节的默认零位
    #    确保 caster/wheel 等关节的 TF 能被 robot_state_publisher 发布
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
        output="log",
    )
```

注意：joint_state_publisher 默认输出到 /joint_states，
planning_robot_state_publisher 订阅 /joint_states，
三者都用同一个话题，不需要 remap。

### 2.2 添加 static_transform_publisher — Robot → base_link

用途：Isaac Sim 的 TF 树根 frame 叫 "Robot"（对应 USD 路径 /World/Robot），
需要桥接到 URDF 的 base_link，让 RViz 的 TF 插件不会报 Robot frame 孤立。

```python
    # ------------------------------------------------------------------
    # 静态 TF: Robot → base_link
    # 桥接 Isaac Sim 的根 frame 名 "Robot" 到 URDF 的 base_link
    # ------------------------------------------------------------------
    static_tf_robot_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot_frame_bridge",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "Robot",
            "--child-frame-id", "base_link",
        ],
        parameters=[{"use_sim_time": True}],
        output="log",
    )
```

### 2.3 修改 nodes 列表

将 nodes 列表替换为：

```python
    nodes = [
        joint_state_publisher_node,       # ← 新增，最先启动，合并关节状态
        planning_robot_state_publisher,   # FK: /joint_states → TF
        odom_to_tf_node,                  # /odom → world→odom→base_link TF
        static_tf_robot_to_base,          # 静态: Robot → base_link
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[ocs2_arm_controller_spawner]),
    ]
```

if/else 部分保持不变（arms_target_manager 和 rviz_node 的添加逻辑不变）。

## 任务3：编译并验证

```bash
cd ~/ros2_ws
colcon build --packages-select r1_description
source install/setup.bash
```

### 验证步骤

```bash
# Step 1: Isaac Sim 处于 Play 状态（已删除 publish_tf）
# 确认 /odom 正在发布
ros2 topic hz /odom   # 应 ~60Hz

# Step 2: 启动
ros2 launch r1_description ocs2_isaac.launch.py

# Step 3: 等待 5s 后验证（无 "jump back in time" 警告）
# 验证 TF 树完整性
ros2 run tf2_ros tf2_echo world base_link
# 预期: Translation 约 (0.175, 3.18, -0.094), Yaw ≈ 180°（来自 /odom）

ros2 run tf2_ros tf2_echo base_link camera_H_color_optical_frame
# 预期: 有稳定输出（head_camera_joint 改 fixed 后 TF 链完整）

ros2 run tf2_ros tf2_echo base_link caster_LB_link
# 预期: 有输出（caster 仍然 continuous，由 joint_state_publisher 补充状态）

# Step 4: 验证 /tf 发布者数量
ros2 topic info /tf --verbose
# 应只有 2-3 个 publisher:
#   1. planning_robot_state_publisher (base_link 子树)
#   2. odom_to_tf (odom → base_link)
#   3. static_transform_publisher (Robot → base_link, 可能在 /tf_static)

# Step 5: RViz 检查
# - 所有 link 显示 "Transform OK"
# - 无闪烁
# - 机器人模型完整显示（包括 camera_H 相机）
# - Interactive Marker 拖动能控制手臂
```

## 关于 /odom 数据的说明

当前 /odom 数据显示：
- position: (0.175, 3.18, -0.094)  ← 机器人实际在仿真中的位置
- orientation yaw ≈ 180°          ← 机器人朝向 -Y 方向

z = -0.094 是 Isaac Sim 底盘原点与 URDF base_link 原点的高度差，
这是正常的（底盘几何中心和 URDF 原点定义不同），暂不需要修复。

odom_to_tf.py 会直接将此数据转为 odom → base_link 的 TF，
这意味着在 RViz 中，机器人不在原点而在 (0.175, 3.18) 位置。
如果需要机器人在 RViz 中显示在原点，可以在 odom_to_tf.py 中减去初始偏置，
但这会破坏底盘移动的正确性，暂不建议修改。
