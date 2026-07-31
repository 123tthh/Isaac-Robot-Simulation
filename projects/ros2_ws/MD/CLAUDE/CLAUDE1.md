# CLAUDE.md — OCS2 ↔ Isaac Sim 仿真闭环

> 给 Claude Code 的项目上下文文件 放置于 `~/ros2_ws/MD/CLAUDE.md`

------

## 项目概述

轮式双臂机器人在 Isaac Sim 5.1 中进行物理仿真，执行托盘抓放任务。

**三阶段流水线：**

1. **阶段1 (当前)**: Isaac Sim + OCS2 仿真闭环 → 生成可行轨迹
2. **阶段2**: 轨迹数据 → 强化学习训练
3. **阶段3**: Sim2Real → 实机部署

**控制架构**: 修改版 OCS2 运行在 ROS 2 Humble 中，Isaac Sim 5.1 通过 ROS 2 bridge 通信。

------

## 机器人硬件配置

| 部件 | 规格                                                        |
| ---- | ----------------------------------------------------------- |
| 底盘 | 轮式移动底盘 (2 驱动轮 + 4 万向轮)                          |
| 躯干 | waist_joint (升降, prismatic) + body_joint (俯仰, revolute) |
| 手臂 | 2× 睿尔曼 RM75-6FB，各 7 DOF                                |
| 夹爪 | 2× 平行夹爪 (PGIA)，开/关离散 (0 和 0.04m)，Mimic 联动      |
| 相机 | 3× Intel RealSense D455                                     |

### /joint_states 中的 27 个关节

```
手臂 (14): left_joint1..7, right_joint1..7
夹爪 (4): left_PGIA_joint1/2, right_PGIA_joint1/2
躯干 (2): waist_joint, body_joint
头部 (1): head_camera_joint (r1_fixed.urdf 中已改 fixed)
轮子 (6): wheel_L/R_joint, caster_LB/LF/RB/RF_joint
```

------

## 文件结构

```
~/ros2_ws/
├── CLAUDE.md
├── src/robot/                    ← 包名: r1_description
│   ├── urdf/r1_fixed.urdf       ← 当前使用的 URDF
│   ├── xacro/ros2_control/robot.xacro  ← 硬件接口定义 (isaac/mock 切换)
│   ├── config/ocs2/task.info     ← OCS2 MPC 配置
│   ├── config/ros2_control/ocs2_controllers.yaml
│   ├── launch/ocs2_isaac.launch.py
│   ├── scripts/odom_to_tf.py     ← /odom → TF
│   └── meshes/                   ← 链接到 /home/gtk/Desktop/Robot/meshes
├── src/arms_ros2_control/
│   ├── topic_based_ros2_control/ ← topic_based 硬件接口插件
│   └── arms_target_manager/      ← RViz Interactive Marker
└── scripts/
    ├── check_isaac_bridge.py     ← 通信诊断
    ├── set_drive_params.py       ← Isaac Sim 关节参数设置 (Script Editor)
    └── robot_teach.py            ← 关节示教工具
```

------

## Isaac Sim Action Graph (当前状态)

```
Arm_Control_Graph:       订阅 /arm_joint_cmd → articulation_controller
Gripper_Control_Graph:   订阅 /left|right_gripper_controller/commands
State_Telemetry_Graph:   发布 /joint_states + /clock (publish_tf 已删除)
Camera_Publish_Graph:    发布 6 个 RGB-D 话题
```

Isaac Sim 只发布数据 (/joint_states, /clock, /odom, 相机)，不发布 TF。 ROS2 侧独占所有 TF 发布。

------

## 🔴 当前阻塞问题 (需要 Claude Code 解决)

### 问题1: robot.xacro isaac 模式未生效

**文件**: ~/ros2_ws/src/robot/xacro/ros2_control/robot.xacro

当前 xacro 条件判断可能语法有问题，导致即使传入 `ros2_control_hardware_type:=isaac`， 实际加载的仍然是 mock_components/GenericSystem。

日志证据: `Successful 'activate' of hardware 'r1_mock_system'` 而不是 `r1_arm_isaac`。

**排查步骤**:

```bash
# 1. 确认 topic_based_ros2_control 包存在
ros2 pkg list | grep topic_based

# 2. 确认 xacro 条件判断生效
xacro ~/ros2_ws/src/robot/xacro/ros2_control/robot.xacro \
  ros2_control_hardware_type:=isaac 2>&1 | grep -i "plugin\|name="

# 3. 如果条件判断不生效，检查 xacro 语法
#    当前写法:
#      <xacro:if value="${'$(arg ros2_control_hardware_type)' == 'isaac'}">
#    可能需要改为:
#      <xacro:arg name="ros2_control_hardware_type" default="mock" />
#      <xacro:property name="hw_type" value="$(arg ros2_control_hardware_type)" />
#      <xacro:if value="${hw_type == 'isaac'}">
```

**这是最关键的问题**: 如果硬件接口是 mock，topic_based_ros2_control 不会 读 /joint_states 也不会写 /arm_joint_cmd，整个 OCS2 → Isaac Sim 控制链路断开。

### 问题2: 托盘位置获取 (不通过相机)

初步任务中不使用相机定位，而是通过 Isaac Sim 直接读取托盘 USD prim 的世界坐标。 需要在 Isaac Sim 中写一个 Script Editor 脚本或 Action Graph 节点来发布托盘位姿到 /tray_pose (geometry_msgs/PoseStamped) 话题。

------

## 已解决问题历史 (共 14 个)

1. URDF 惯性参数修复 (RM75 官方数据)
2. trunk_link 惯量单位错误
3. D455 相机惯量修正
4. waist/body_joint effort=0 修正
5. 关节 Stiffness/Damping 欠阻尼修正
6. Isaac Sim 最小闭环验证 (手臂能响应 /arm_joint_cmd)
7. use_sim_time spawner 超时 → 添加 ROS2PublishClock
8. libarms_target_manager.so → CMakeLists install 路径修正
9. URDF mesh 路径 → package://r1_description/meshes/
10. TF 重映射 → 删除 /tf→/ocs2_tf
11. TF 双源冲突 → 删除 Isaac Sim publish_tf
12. odom_to_tf 时间戳 sec=0 → 延迟到收到 /odom 再发布
13. head_camera_joint → fixed
14. lidar_joint → fixed

------

## 优先级

```
🔴 P0: 修复 robot.xacro 使 topic_based_ros2_control 生效
🔴 P0: 验证 /arm_joint_cmd 数据流 (OCS2 → Isaac Sim)
🟡 P1: RViz Interactive Marker → 手臂跟随
🟡 P1: 重新运行 set_drive_params.py 并保存 Stage
🟢 P2: 夹爪联调 + 完整阶段 A/B/C
```



































## 三
