# Arms MoveIt

本仓库存储了多种机械臂的MoveIt配置和一些通用的启动程序。机器人描述文件位于 [`robot_descriptions`](https://github.com/fiveages-sim/robot_descriptions) 包中。为了增强控制功能，您可以选择性地包含 [`arms_ros2_control`](https://github.com/fiveages-sim/arms_ros2_control) 包。

**基础设置：**
```
ros2_ws
├── src
│   ├── robot_descriptions
│   └── arms_moveit
```

**增强设置（推荐）：**
```
ros2_ws
├── src
│   ├── robot_descriptions
│   ├── arms_moveit
│   └── arms_ros2_control  # 可选：增强控制包
```

## 1. MoveIt2 控制

### 1.1 MoveIt2 环境设置

请参考 [MoveIt2 入门指南](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html#install-ros-2-and-colcon) 获取安装说明。

### 1.2 尝试首次启动

设置好MoveIt2环境后，您可以构建和测试moveit_config包。

* 构建Dobot CR5 MoveIt2包
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to cr5_moveit_config --symlink-install
    ```
* 使用模拟组件启动
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py robot:=cr5
    ```
  ![moveit](.images/moveit_dobot.png)

## 2. 在Gazebo中使用MoveIt2

要在Gazebo中启动MoveIt2，您需要安装Gazebo Harmonic。您可以选择默认的ROS2 Gazebo集成或来自 `arms_ros2_control` 的增强版本。

**选项1：默认ROS2 Gazebo集成**

* 使用标准ROS2 Gazebo集成
  ```bash
  sudo apt-get install ros-humble-gz-ros2-control
  ```

**选项2：增强版gz_ros2_control（推荐）**

* 安装Gazebo Harmonic (ROS2 Humble)
    ```bash
    sudo apt-get install ros-humble-ros-gzharmonic
    ```
* 增强版 `gz_ros2_control` 包包含在 `arms_ros2_control` 项目中。确保您有完整的项目结构：
    ```
    ros2_ws
    ├── src
    │   ├── robot_descriptions
    │   ├── arms_moveit
    │   └── arms_ros2_control
    │       └── hardwares
    │           └── gz_ros2_control  # (为ROS2 Humble + Gazebo Harmonic修改)
    ```
* 编译增强版gz_ros2_control包
    ```bash
    cd ~/ros2_ws
    colcon build --packages-up-to gz_ros2_control --symlink-install
    ```

* 在Gazebo中启动MoveIt2
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py robot:=cr5 hardware:=gz
    ```

  ![moveit gz](.images/moveit_dobot_gz.png)

## 3. 在IsaacSim中使用MoveIt2

要在Isaac Sim中启动MoveIt2，您可以选择默认的topic-based集成或来自 `arms_ros2_control` 的增强版本。

**选项1：默认Topic-based集成**

* 使用标准topic-based集成
    ```bash
    sudo apt-get install ros-humble-topic-based-ros2-control
    ```

**选项2：增强版topic_based_ros2_control（推荐）**

* 增强版 `topic_based_ros2_control` 包包含在 `arms_ros2_control` 项目中。确保您有完整的项目结构：
  ```
  ros2_ws
  ├── src
  │   ├── robot_descriptions
  │   ├── arms_moveit
  │   └── arms_ros2_control
  │       └── hardwares
  │           └── topic_based_ros2_control  # 增强topic-based集成
  ```
* 编译增强版topic_based_ros2_control包
  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to topic_based_ros2_control --symlink-install
  ```

* 在Isaac Sim中启动MoveIt2（在此步骤之前启动Isaac Sim）
    ```bash
    source ~/ros2_ws/install/setup.bash
    ros2 launch moveit_common_config demo.launch.py hardware:=isaac robot:=cr5
    ```

  ![moveit isaac](.images/moveit_dobot_isaac.png)

**注意**：增强版 `topic_based_ros2_control` 包提供了改进的安全功能，包括使用机器人当前位置作为初始指令位置的能力。这可以防止启动时的突然运动，对真实硬件应用特别重要。该包包含可配置参数 `initialize_commands_from_state` 来控制此行为。

## 4. 使用MoveIt2 Servo进行遥操作

> **如何使用手柄控制**:
> * `A` 键用于在基座和末端执行器之间切换坐标系。
> * `X` 键用于触发夹爪。

* 编译moveit teleop
  ```bash
  cd ~/ros2_ws
  colcon build --packages-up-to moveit_teleop --symlink-install
  ```
* 模拟组件
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config servo.launch.py robot:=cr5 
  ```
* Gazebo
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config servo.launch.py hardware:=gz robot:=cr5 
  ```
* Isaac Sim
  ```bash
  source ~/ros2_ws/install/setup.bash
  ros2 launch moveit_common_config servo.launch.py hardware:=isaac robot:=cr5 
  ``` 