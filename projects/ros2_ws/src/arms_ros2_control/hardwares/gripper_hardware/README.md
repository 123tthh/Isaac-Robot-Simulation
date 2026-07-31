# Gripper Hardware Interface

基于ROS2话题的夹爪硬件接口，用于ros2_control框架。

## 功能特性

- ✅ 通过ROS2话题订阅夹爪状态信息
- ✅ 通过ROS2话题发布夹爪控制指令
- ✅ 支持位置、速度、力矩接口
- ✅ 可配置的命令发布阈值
- ✅ 支持从首次状态自动初始化命令

## 安装

```bash
cd ~/arm_ws
colcon build --packages-select gripper_hardware
source install/setup.bash
```

## 使用方法

### 1. 在URDF中配置硬件

```xml
<ros2_control name="gripper" type="system">
  <hardware>
    <plugin>gripper_hardware/GripperHardware</plugin>
    <param name="state_topic">/gripper_state</param>
    <param name="command_topic">/gripper_command</param>
    <param name="command_threshold">0.001</param>
    <param name="initialize_commands_from_state">true</param>
  </hardware>
  
  <joint name="gripper_finger_joint">
    <command_interface name="position">
      <param name="min">0.0</param>
      <param name="max">0.085</param>
    </command_interface>
    <state_interface name="position">
      <param name="initial_value">0.0</param>
    </state_interface>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
  </joint>
</ros2_control>
```

### 2. 配置参数说明

| 参数名称                         | 类型   | 默认值             | 说明                                       |
| -------------------------------- | ------ | ------------------ | ------------------------------------------ |
| `state_topic`                    | string | `/gripper_state`   | 订阅夹爪状态的话题名称                     |
| `command_topic`                  | string | `/gripper_command` | 发布夹爪命令的话题名称                     |
| `command_threshold`              | double | `0.001`            | 命令发布阈值，只有当命令变化超过此值才发布 |
| `initialize_commands_from_state` | bool   | `true`             | 是否从首次接收的状态初始化命令值           |

### 3. 话题格式

#### 状态话题 (订阅)
```bash
# 话题类型: sensor_msgs/msg/JointState
# 示例发布:
ros2 topic pub /gripper_state sensor_msgs/msg/JointState "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  name: ['gripper_finger_joint'],
  position: [0.04],
  velocity: [0.0],
  effort: [0.0]
}"
```

#### 命令话题 (发布)
```bash
# 话题类型: sensor_msgs/msg/JointState
# 订阅查看:
ros2 topic echo /gripper_command
```

### 4. 示例launch文件

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[{'robot_description': robot_description_content}],
            output='screen',
        ),
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['gripper_controller'],
            output='screen',
        ),
    ])
```

### 5. 控制器配置示例

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    gripper_controller:
      type: forward_command_controller/ForwardCommandController

gripper_controller:
  ros__parameters:
    joints:
      - gripper_finger_joint
    interface_name: position
```

## 测试

### 测试状态订阅

1. 启动ros2_control节点（加载此硬件接口）
2. 发布状态消息:
```bash
ros2 topic pub /gripper_state sensor_msgs/msg/JointState "{
  name: ['gripper_finger_joint'],
  position: [0.05]
}" --rate 10
```

3. 查看状态是否被正确读取:
```bash
ros2 topic echo /joint_states
```

### 测试命令发布

1. 发送控制命令:
```bash
ros2 topic pub /gripper_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.08]}" --once
```

2. 查看命令是否被发布:
```bash
ros2 topic echo /gripper_command
```

## 架构说明

```
┌─────────────────────────────────────────────┐
│           ros2_control Framework            │
│  ┌───────────────────────────────────────┐  │
│  │      Controller Manager               │  │
│  │  ┌─────────────────────────────────┐  │  │
│  │  │   Gripper Controller            │  │  │
│  │  └────────────┬────────────────────┘  │  │
│  └───────────────┼───────────────────────┘  │
│                  │                           │
│  ┌───────────────▼───────────────────────┐  │
│  │   GripperHardware Interface          │  │
│  │                                       │  │
│  │   read()  ◄─┐         ┌─► write()    │  │
│  └─────────────┼─────────┼───────────────┘  │
└────────────────┼─────────┼───────────────────┘
                 │         │
      /gripper_state    /gripper_command
         (订阅)           (发布)
                 │         │
        ┌────────▼─────────▼────────┐
        │   真实夹爪硬件/模拟器      │
        └────────────────────────────┘
```

## 工作原理

1. **初始化 (on_init)**:
   - 读取配置参数
   - 创建ROS2节点
   - 创建状态订阅者和命令发布者
   - 初始化关节状态和命令存储

2. **读取状态 (read)**:
   - 处理订阅队列中的消息
   - 从最新状态消息更新内部状态
   - 如果启用，首次接收状态时初始化命令值

3. **写入命令 (write)**:
   - 检查命令是否有显著变化（超过阈值）
   - 如果有变化，将命令打包成JointState消息并发布

## 常见问题

### Q: 夹爪不动作？
A: 
- 检查状态话题是否有数据发布: `ros2 topic hz /gripper_state`
- 检查命令话题是否有订阅者: `ros2 topic info /gripper_command`
- 确认控制器已正确加载和启动

### Q: 如何调整命令发布频率？
A: 调整`command_threshold`参数，值越小发布越频繁

### Q: 启动时夹爪突然移动？
A: 设置`initialize_commands_from_state`为`true`，这样命令会从首次接收的状态初始化，避免突然移动

## 许可证

MIT License

## 贡献

欢迎提交Issue和Pull Request！
