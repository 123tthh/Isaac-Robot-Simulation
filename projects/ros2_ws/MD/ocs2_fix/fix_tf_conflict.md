# Claude Code 任务：修复 TF 双源冲突（时间倒流警告）

## 背景
参考 ~/ros2_ws/CLAUDE.md 了解完整项目上下文。

## 根本原因
Isaac Sim 的 `publish_tf` 节点和 ROS2 的 `planning_robot_state_publisher`
同时在发布 base_link 及其子 frame 的 TF，时间戳微小差异导致 tf2_buffer
频繁判定时间倒流并清空缓存，造成 RViz 闪烁和相机 frame 丢失。

## 解决策略：路线A — planning_robot_state_publisher 独占 TF 树

TF 职责划分：
- planning_robot_state_publisher → 发布完整 TF 树（base_link 及所有子 frame）
- Isaac Sim publish_tf → 删除，不再发布任何 TF
- odom_to_tf 节点 → 发布 world → odom → base_link（动态）
- static_transform_publisher → 发布 Robot → base_link（Isaac Sim 根 frame 桥接）

## 步骤1：在 Isaac Sim 中删除 publish_tf 节点

用户需要在 Isaac Sim 中手动操作（无法自动化）：

1. 打开 State_Telemetry_Graph（Window → Visual Scripting → Action Graph）
2. 找到 `publish_tf` 节点（类型：isaacsim.ros2.bridge.ROS2PublishTransformTree）
3. 右键 → Delete 删除该节点
4. 确认 `ros2_publish_joint_state` 节点保留（我们仍然需要 /joint_states）
5. 保存 Stage

删除后 State_Telemetry_Graph 应只剩：
- on_playback_tick
- ros2_context
- read_sim_time
- ros2_publish_joint_state（保留）
- ros2_publish_clock（保留）

## 步骤2：修复 r1_fixed.urdf 中 head_camera_joint 的问题

从截图看 head_camera_link 的 TF 断了。这通常是 URDF 中该关节是
continuous 类型但 joint_states 里没有对应名称，或者关节名称不匹配。

检查 URDF 中 head_camera_joint 的定义：
```bash
grep -A5 "head_camera_joint" ~/ros2_ws/src/r1_description/urdf/r1_fixed.urdf
```

如果关节名是 `head_camera_joint` 但 /joint_states 里叫别的名字，
需要统一命名。检查 /joint_states 中的实际名称：
```bash
ros2 topic echo /joint_states --once 2>/dev/null | grep -A1 "name"
```

## 步骤3：处理 caster_*_link 的 TF 缺失

截图中 caster_LB_link、caster_LF_link、caster_RB_link、caster_RF_link 也断了。
这些万向轮关节在 /joint_states 里通常有值，但 URDF 里的关节名可能不匹配。

检查 URDF 中 caster 关节名：
```bash
grep "caster.*joint\|joint.*caster" ~/ros2_ws/src/r1_description/urdf/r1_fixed.urdf | grep "name="
```

检查 /joint_states 中的 caster 关节名：
```bash
ros2 topic echo /joint_states --once 2>/dev/null | python3 -c "
import sys, yaml
data = yaml.safe_load(sys.stdin.read())
for n in data.get('name', []):
    if 'caster' in n or 'head' in n:
        print(n)
"
```

如果名称匹配但仍然断了，说明 URDF 里这些关节是 fixed 类型（不需要 /joint_states），
可以将它们在 URDF 中改为 `type="fixed"`：

```bash
# 在 r1_fixed.urdf 中找到 caster 关节并检查类型
grep -B2 -A10 'name="caster_LB_joint"' ~/ros2_ws/src/r1_description/urdf/r1_fixed.urdf
```

如果 caster 关节应该是 continuous/revolute（可以自由旋转的万向轮），
则需要在 launch 文件中添加一个 joint_state_publisher 来发布这些关节的零位状态：

在 ocs2_isaac.launch.py 中添加：
```python
    # 为不受控关节发布默认零位关节状态（caster 轮、head_camera 等）
    # 防止 robot_state_publisher 因缺少这些关节状态而断开 TF
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="auxiliary_joint_state_publisher",
        parameters=[
            {"use_sim_time": True},
            {"source_list": ["/joint_states"]},  # 合并来自 Isaac Sim 的关节状态
            {"rate": 60},
        ],
        output="log",
    )
```

这个节点会把 Isaac Sim 的 /joint_states（27个关节）和 URDF 中剩余
未被 /joint_states 覆盖的关节（默认为0）合并后重发，确保 
planning_robot_state_publisher 能算完整的 TF 树。

把它加到 nodes 列表的最前面：
```python
    nodes = [
        joint_state_publisher_node,   # ← 新增，必须最先启动
        planning_robot_state_publisher,
        odom_to_tf_node,
        static_tf_robot,
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[ocs2_arm_controller_spawner]),
    ]
```

## 步骤4：处理 camera_H_* frame 断裂问题

camera_H_link 及其子 frame（color_frame、depth_frame 等）断了，
但 camera_L_link 和 camera_R_link 正常。

这说明 head_camera_joint → camera_H_joint 这条链上有断点。
很可能是 head_camera_link 断了导致整个子树都断。

检查 URDF 中的 head_camera 相关关节链：
```bash
grep -E "head_camera|camera_H" ~/ros2_ws/src/r1_description/urdf/r1_fixed.urdf | grep "joint\|parent\|child"
```

如果确认是 head_camera_joint 的关节状态缺失导致链断，
joint_state_publisher（步骤3添加的）会自动补充零位状态修复它。

## 步骤5：验证

重新编译（如果修改了 CMakeLists.txt）：
```bash
cd ~/ros2_ws
colcon build --packages-select r1_description
source install/setup.bash
```

启动并验证：
```bash
# 终端1: Isaac Sim 已 Play（已删除 publish_tf 节点）
# 验证 /tf 话题只有一个发布者
ros2 topic info /tf

# 终端2: 启动 OCS2
ros2 launch r1_description ocs2_isaac.launch.py

# 验证无时间跳跃警告（等待30秒，不应再出现 "Detected jump back in time"）
# 验证 TF 树完整
ros2 run tf2_ros tf2_echo world left_Link7
ros2 run tf2_ros tf2_echo world right_Link7

# 验证 camera_H 链路
ros2 run tf2_ros tf2_echo base_link camera_H_color_frame
```

## 预期结果

- RViz 不再闪烁
- 所有 link 显示 "Transform OK"（包括 camera_H_*、caster_*、head_camera_link）
- `/tf` 话题只有 planning_robot_state_publisher + odom_to_tf 两个发布者
- Isaac Sim 的 /joint_states 仍然正常发布（publish_joint_state 节点保留）
- 拖动 RViz Interactive Marker 能控制手臂（OCS2 闭环正常）

## 关于 world → base_link 的偏移

tf2_echo 输出显示：
```
Translation: [0.000, -0.000, -0.094]
Pitch: 0.185°
```

Z=-0.094m 和微小 pitch 说明 /odom 数据包含了一个固定偏置（可能是
Isaac Sim 的底盘原点与 URDF base_link 原点不重合导致的）。

这是正常的，但如果影响 OCS2 的末端位姿计算，可以在 odom_to_tf.py 中
添加一个偏置补偿，或者在 Isaac Sim 的 URDF 导入时检查 base_link 的位置。

暂时不需要修复，等 OCS2 闭环验证完毕后再处理。
