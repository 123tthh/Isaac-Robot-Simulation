#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Controller configuration
    package_path = get_package_share_directory("rm_hardware")
    config_file = os.path.join(package_path, "config", "rm_75_ros2_control.yaml")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("rm_hardware"),
                    "urdf",
                    "rm_75.xacro",
                ]
            ),
        ]
    )
    robot_description_content = ParameterValue(robot_description_content, value_type=str)

    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description,
            config_file,
        ],
        output="both",
    )

    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Define admittance controller spawner
    admittance_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["rm_admittance_controller", "--controller-manager", "/controller_manager"],
    )

    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["trajectory_controller", "--controller-manager", "/controller_manager"],
    )

    position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_controller", "--controller-manager", "/controller_manager"],
    )

    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["force_torque_sensor_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Event handler to launch trajectory_controller_spawner after admittance_controller_spawner finishes
    trajectory_spawner_after_admittance = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=admittance_controller_spawner,
            on_exit=[trajectory_controller_spawner],
        )
    )

    position_spawner_after_admittance = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=admittance_controller_spawner,
            on_exit=[position_controller_spawner],
        )
    )

    rviz_config_file = PathJoinSubstitution([FindPackageShare("rm_hardware"), "rviz", "rm_75_display.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    nodes_to_start = [
        robot_state_publisher,
        control_node,
        joint_state_broadcaster_spawner,
        force_torque_sensor_broadcaster_spawner,
        admittance_controller_spawner,  # Start this one first
        trajectory_spawner_after_admittance,  # This will trigger the position spawner
        # position_spawner_after_admittance,
        rviz_node,
    ]

    return LaunchDescription(nodes_to_start)
