from launch import LaunchDescription
from launch_ros.actions import Node
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    robot_description_urdf = os.path.join(get_package_share_directory("r1_description"), "xacro", "r1.xacro")
    robot_description = xacro.process_file(robot_description_urdf).toxml()

    rviz_config_file = os.path.join(get_package_share_directory("r1_description"), "rviz", "urdf.rviz")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_ursf_show",
                output="screen",
                arguments=["-d", rviz_config_file],
            ),
        ]
    )
