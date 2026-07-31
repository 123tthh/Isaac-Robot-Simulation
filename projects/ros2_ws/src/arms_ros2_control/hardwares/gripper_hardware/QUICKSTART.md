# 夹爪硬件接口快速使用指南

## 快速开始

### 1. 编译包

```bash
cd /home/bt/Desktop/arm_ws
colcon build --packages-select gripper_hardware
source install/setup.bash
```

### 2. 在URDF中使用

在你的机器人URDF文件中添加以下配置：

```xml
<ros2_control name="gripper" type="system">
  <hardware>
    <plugin>gripper_hardware/GripperHardware</plugin>
    <param name="state_topic">/gripper_state</param>
    <param name="command_topic">/gripper_command</param>
  </hardware>
  
  <joint name="gripper_finger_joint">
    <command_interface name="position"/>
    <state_interface name="position"/>
  </joint>
</ros2_control>
```

### 3. 测试硬件接口

#### 方法1: 使用ros2 topic手动测试

终端1 - 发布状态信息（模拟真实硬件）:
```bash
ros2 topic pub /gripper_state sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}},
  name: ['gripper_finger_joint'],
  position: [0.04],
  velocity: [0.0],
  effort: [0.0]
}" --rate 10
```

终端2 - 启动你的ros2_control节点（包含gripper_hardware）

终端3 - 监听命令（查看硬件接口发布的命令）:
```bash
ros2 topic echo /gripper_command
```

终端4 - 发送控制命令:
```bash
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.08]}" --once
```

#### 方法2: 使用提供的Python模拟器

```bash
# 运行夹爪模拟器（模拟真实硬件行为）
python3 gripper_simulator.py
```

## 核心文件说明

### 头文件
- `include/gripper_hardware/gripper_hardware.hpp` - 硬件接口类声明

### 源文件
- `src/gripper_hardware.cpp` - 硬件接口实现

### 配置文件
- `gripper_hardware_plugin.xml` - 插件描述文件

## 工作流程

```
控制器 → write() → 发布/gripper_command → 真实硬件/模拟器
                                              ↓
控制器 ← read() ← 订阅/gripper_state ← 真实硬件/模拟器
```

## 支持的接口

- ✅ position (位置) - 必须
- ✅ velocity (速度) - 可选
- ✅ effort (力矩) - 可选

## 配置参数

| 参数                           | 默认值           | 说明             |
| ------------------------------ | ---------------- | ---------------- |
| state_topic                    | /gripper_state   | 状态订阅话题     |
| command_topic                  | /gripper_command | 命令发布话题     |
| command_threshold              | 0.001            | 命令发布阈值     |
| initialize_commands_from_state | true             | 从状态初始化命令 |

## 常见问题排查

### 问题: 编译失败
```bash
# 清理并重新编译
cd /home/bt/Desktop/arm_ws
rm -rf build/gripper_hardware install/gripper_hardware
colcon build --packages-select gripper_hardware --cmake-clean-cache
```

### 问题: 插件未找到
```bash
# 确保sourced环境
source /home/bt/Desktop/arm_ws/install/setup.bash

# 检查插件是否可用
ros2 pkg list | grep gripper_hardware

# 列出可用的硬件接口插件
ros2 plugin list --list-plugin hardware_interface::SystemInterface
```

### 问题: 话题无数据
```bash
# 检查话题
ros2 topic list

# 检查话题频率
ros2 topic hz /gripper_state
ros2 topic hz /gripper_command

# 检查话题信息
ros2 topic info /gripper_state
```

## 下一步

1. 将此硬件接口集成到你的机器人URDF中
2. 配置合适的控制器（如forward_command_controller或joint_trajectory_controller）
3. 连接到你的真实硬件或模拟器
4. 根据需要调整参数

更多详细信息请查看 `README.md`
