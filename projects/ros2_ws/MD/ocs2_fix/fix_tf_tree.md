# Claude Code 任务：修复 TF 树缺失变换

## 文件位置
修改文件: `~/ros2_ws/src/r1_description/launch/ocs2_isaac.launch.py`
当前文件内容参见项目中已有的版本。

## 问题
RViz 中 TF 树报两个缺失变换:
1. `No transform from [world] to [base_link]` — OCS2 的 arms_target_manager 需要 world frame
2. `No transform from [Robot] to [base_link]` — Isaac Sim publish_tf 发布的根 frame 叫 "Robot"

## 解决方案
在 launch 文件的 `launch_setup` 函数中，在 `nodes` 列表组装之前，添加两个 static_transform_publisher 节点。

### 修改1: 添加 world → base_link 静态变换

在 `# Assemble node list` 注释之前，添加:

```python
    # ------------------------------------------------------------------
    # 7. Static TF: world → base_link (驻车模式, 机器人固定不动)
    #    如果底盘会移动(阶段B), 需要替换为动态 TF 发布器
    # ------------------------------------------------------------------
    static_tf_world = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_base",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "world",
            "--child-frame-id", "base_link",
        ],
        parameters=[{"use_sim_time": True}],
        output="log",
    )
```

### 修改2: 添加 Robot → base_link 静态变换

Isaac Sim 的 publish_tf 节点会发布一个叫 "Robot" 的根 frame (对应 USD 路径 /World/Robot)。
需要桥接到 URDF 的 base_link:

```python
    # ------------------------------------------------------------------
    # 8. Static TF: Robot → base_link (桥接 Isaac Sim TF 树)
    #    Isaac Sim publish_tf 的根 frame 名为 "Robot"
    #    映射到 URDF 的 base_link
    # ------------------------------------------------------------------
    static_tf_robot = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="robot_to_base",
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

### 修改3: 将这两个节点加入 nodes 列表

修改 `nodes` 列表，在 `planning_robot_state_publisher` 之后加入:

```python
    nodes = [
        planning_robot_state_publisher,
        static_tf_world,       # ← 新增
        static_tf_robot,       # ← 新增
        ros2_control_node,
        TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=3.0, actions=[ocs2_arm_controller_spawner]),
    ]
```

## 注意

- 如果 Isaac Sim 的 `publish_tf` 节点同时也在发布 `base_link` 及其子 frame 的 TF, 
  可能会和 `planning_robot_state_publisher` 产生冲突(两个源同时发布同一个变换)。
  
  如果出现 TF 冲突警告 "TF_REPEATED_DATA" 或 "TF has two or more unconnected trees",
  需要在 Isaac Sim 的 State_Telemetry_Graph 中设置 publish_tf 节点的 Target Prim 
  只包含 `/World/Robot` 的直接子级(而不是整个树), 或者直接删除 Isaac Sim 侧的 publish_tf,
  让 planning_robot_state_publisher 独占 TF 发布。

- `world → base_link` 变换在驻车模式下是 identity (0,0,0)。
  阶段B底盘移动时, 需要替换为动态发布(从 /odom 或 /agv_pose 话题读取位姿)。

