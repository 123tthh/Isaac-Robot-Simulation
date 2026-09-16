"""
OCS2 + Isaac Sim 闭环控制 launch 文件
R1 双臂机器人 (14-DOF, 停车模式)

本地文档参考:
  https://docs.ros.org/en/rolling/p/controller_manager/doc/userdoc.md
  https://docs.ros.org/en/rolling/p/robot_state_publisher/__README.md
  https://docs.ros.org/en/rolling/p/xacro/xacro.cli.md

前提条件:
  - Isaac Sim 已按下 Play，State_Telemetry_Graph 发布 /isaac_joint_states
  - Arm_Control_Graph 已订阅 /arm_joint_cmd
  - /odom 为可选输入；当前场景没有 /odom publisher，odom_to_tf 会等待该话题

启动方式:
  ros2 launch r1_description ocs2_isaac.launch.py

数据流:
  Isaac Sim
    ├── /odom (可选，nav_msgs/Odometry)
    │     └──> odom_to_tf → TF: world→odom (static) + odom→base_link (dynamic)
    └── /isaac_joint_states (27 joints)
          └──> joint_state_fill 补齐辅助关节 → /joint_states
                └──> topic_based_ros2_control (读 left/right_joint1-7)
                └──> ocs2_arm_controller (MPC 规划)
                      └──> topic_based_ros2_control (写命令)
                            └──> /arm_joint_cmd (sensor_msgs/JointState)
                                  └──> Isaac Sim Arm_Control_Graph → 机械臂运动

TF 树:
  world (固定)
    └── odom (有 /odom 时由 odom_to_tf 发布静态 identity)
          └── base_link (有 /odom 时动态更新)
                └── trunk_link → 手臂/头部/轮子
"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch_ros.actions import Node


ROBOT_PKG = "r1_description"


def launch_setup(context, *args, **kwargs):
    enable_rviz = context.launch_configurations.get("enable_rviz", "true").lower() == "true"
    enable_target_manager = (
        context.launch_configurations.get("enable_target_manager", "true").lower() == "true"
    )
    isaac_joint_states_topic = context.launch_configurations.get(
        "isaac_joint_states_topic", "/isaac_joint_states"
    )
    control_start_delay = float(
        context.launch_configurations.get("control_start_delay", "4.0")
    )

    pkg_path = get_package_share_directory(ROBOT_PKG)

    # ------------------------------------------------------------------
    # 1. robot_description for ros2_control (arm joints only via topic_based)
    # ------------------------------------------------------------------
    xacro_file = os.path.join(pkg_path, "xacro", "ros2_control", "robot.xacro")
    ros2_control_robot_description_content = xacro.process_file(
        xacro_file,
        mappings={"ros2_control_hardware_type": "isaac"},
    ).toxml()
    ros2_control_robot_description = {
        "robot_description": ros2_control_robot_description_content
    }

    # ------------------------------------------------------------------
    # 2. robot_description for runtime TF and OCS2 Visualizer
    #    Uses r1_fixed.urdf (full robot, for FK/visualization)
    # ------------------------------------------------------------------
    planning_urdf_path = os.path.join(pkg_path, "urdf", "r1_fixed.urdf")
    with open(planning_urdf_path, "r") as f:
        robot_description = f.read()

    planning_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="planning_robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "publish_frequency": 100.0,
                "ignore_timestamp": True,
                "use_sim_time": True,
            },
        ],
        remappings=[
            ("/robot_description", "/ocs2_robot_description"),
            ("/joint_states", "/joint_states"),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                # Jazzy controller_manager subscribes to the transient-local
                # robot_description topic. Publish the expanded control xacro
                # here so the received URDF contains <ros2_control>.
                "robot_description": ros2_control_robot_description_content,
                "publish_frequency": 100.0,
                "ignore_timestamp": True,
                "use_sim_time": True,
            },
        ],
    )

    # ------------------------------------------------------------------
    # 0. joint_state_fill
    #    合并 Isaac Sim 原始 joint states 与 URDF 未覆盖关节的默认零位
    #    确保 waist/body/caster/wheel/gripper 等关节的 TF 能被发布
    # ------------------------------------------------------------------
    joint_state_fill_node = Node(
        package="r1_description",
        executable="joint_state_fill.py",
        name="joint_state_fill",
        parameters=[
            {
                "input_topic": isaac_joint_states_topic,
                "output_topic": "/joint_states",
                "publish_rate": 100.0,
                "use_sim_time": False,
            },
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # 3. Dynamic TF: world → odom → base_link
    #    从 Isaac Sim 的 /odom 话题读取底盘位姿，转为 TF
    #    阶段A/C驻车时不变，阶段B底盘移动时实时更新
    # ------------------------------------------------------------------
    odom_to_tf_node = Node(
        package="r1_description",
        executable="odom_to_tf.py",
        name="odom_to_tf",
        output="log",
        parameters=[{"use_sim_time": True}],
    )

    # ------------------------------------------------------------------
    # 静态 TF: Robot → base_link
    # 桥接 Isaac Sim 的根 frame 名 "Robot" 到 URDF 的 base_link
    # ------------------------------------------------------------------
    
#    static_tf_robot_to_base = Node(
#        package="tf2_ros",
#        executable="static_transform_publisher",
#        name="robot_frame_bridge",
#        arguments=[
#           "--x", "0", "--y", "0", "--z", "0",
#           "--roll", "0", "--pitch", "0", "--yaw", "0",
#           "--frame-id", "Robot",
#            "--child-frame-id", "base_link",
#        ],
#       parameters=[{"use_sim_time": True}],
#        output="log",
#    )
    



#    TimerAction(
#        period=1.0,
#        actions=[static_tf_robot_to_base],
#    )
    # ------------------------------------------------------------------
    # 4. ros2_control node
    #    robot_description → topic_based_ros2_control 知道管理哪 14 个关节
    #    ocs2_controllers.yaml → 注册 ocs2_arm_controller
    # ------------------------------------------------------------------
    controllers_yaml = os.path.join(
        pkg_path, "config", "ros2_control", "ocs2_controllers.yaml"
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_control_robot_description,
            controllers_yaml,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # ------------------------------------------------------------------
    # 5. Controller spawners
    # ------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager-timeout", "30"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    ocs2_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ocs2_arm_controller", "--controller-manager-timeout", "30"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

# ------------------------------------------------------------------
    # 6. ArmsTargetManager — interactive end-effector goal via RViz markers
    # ------------------------------------------------------------------
    arms_target_manager_node = None
    if enable_target_manager:
        try:
            from ament_index_python.packages import get_package_share_directory as gpsd
            gpsd("arms_target_manager")
            arms_target_manager_node = Node(
                package="arms_target_manager",
                executable="arms_target_manager_node",
                name="arms_target_manager",
                output="screen",
                parameters=[
                    {
                        "dual_arm_mode": True,
                        "marker_fixed_frame": "base_link",
                        "control_base_frame": "base_link", # <--- 修改 1：将 "world" 改为 "base_link"
                        "enable_vr": False,
                        "use_sim_time": True,
                    },
                ],
                # <--- 修改 2：新增 remappings，强制对齐 OCS2 的输入话题
                remappings=[
                    ("/arms_target_manager/target_pose/left", "/left_target/stamped"),
                    ("/arms_target_manager/target_pose/right", "/right_target/stamped"),
                ]
            )
        except Exception as e:
            print(f"[WARN] arms_target_manager not found, skipping: {e}")

    # ------------------------------------------------------------------
    # 7. RViz2 (optional)
    # ------------------------------------------------------------------
    rviz_node = None
    if enable_rviz:
        rviz_config = os.path.join(
            get_package_share_directory("ocs2_arm_controller"), "config", "demo.rviz"
        )
        rviz_node = Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            output="log",
        )

    # ------------------------------------------------------------------
    # Assemble node list
    # ------------------------------------------------------------------
    nodes = [
        joint_state_fill_node,
        robot_state_publisher,
        planning_robot_state_publisher,
        odom_to_tf_node,
        # static_tf_robot_to_base,
        TimerAction(period=control_start_delay, actions=[ros2_control_node]),
        TimerAction(period=control_start_delay + 2.0, actions=[joint_state_broadcaster_spawner]),
        TimerAction(period=control_start_delay + 3.0, actions=[ocs2_arm_controller_spawner]),
    ]

    if arms_target_manager_node is not None:
        nodes.append(TimerAction(period=control_start_delay + 5.0, actions=[arms_target_manager_node]))

    if rviz_node is not None:
        nodes.append(TimerAction(period=control_start_delay + 5.0, actions=[rviz_node]))

    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_rviz",
                default_value="true",
                description="Launch RViz2 for visualization",
            ),
            DeclareLaunchArgument(
                "enable_target_manager",
                default_value="true",
                description="Launch ArmsTargetManager for interactive EE target control",
            ),
            DeclareLaunchArgument(
                "isaac_joint_states_topic",
                default_value="/isaac_joint_states",
                description="Raw Isaac Sim joint state topic consumed by joint_state_fill",
            ),
            DeclareLaunchArgument(
                "control_start_delay",
                default_value="4.0",
                description="Delay ros2_control startup so /joint_states can latch the current Isaac pose",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
