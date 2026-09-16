# Reference: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.md
# Reference: https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.md

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    dataset_path = LaunchConfiguration("dataset_path").perform(context)
    urdf_path = LaunchConfiguration("urdf_path").perform(context)
    mesh_root = LaunchConfiguration("mesh_root").perform(context)
    episode_index = int(LaunchConfiguration("episode_index").perform(context))
    field = LaunchConfiguration("field").perform(context)
    pose_field = LaunchConfiguration("pose_field").perform(context)
    speed = float(LaunchConfiguration("speed").perform(context))
    publish_rate = float(LaunchConfiguration("publish_rate").perform(context))
    enable_rviz = LaunchConfiguration("enable_rviz").perform(context).lower() == "true"

    robot_description = Path(urdf_path).read_text()
    mesh_root_uri = Path(mesh_root).expanduser().resolve().as_uri() + "/"
    robot_description = robot_description.replace(
        "package://r1_description/meshes/",
        mesh_root_uri,
    )
    package_share = get_package_share_directory("r1_lerobot_sim")
    rviz_config = str(Path(package_share) / "rviz" / "r1_lerobot_replay.rviz")

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": robot_description,
                    "publish_frequency": publish_rate,
                    "ignore_timestamp": True,
                    "use_sim_time": False,
                }
            ],
        ),
        Node(
            package="r1_lerobot_sim",
            executable="joint_state_replay",
            name="joint_state_replay",
            output="screen",
            parameters=[
                {
                    "dataset_path": dataset_path,
                    "urdf_path": urdf_path,
                    "episode_index": episode_index,
                    "field": field,
                    "publish_rate": publish_rate,
                    "speed": speed,
                    "loop": True,
                    "gripper_mode": "episode_minmax",
                    "gripper_closed": 0.04,
                    "gripper_open": 0.12,
                }
            ],
        ),
        Node(
            package="r1_lerobot_sim",
            executable="pose_path_replay",
            name="pose_path_replay",
            output="screen",
            parameters=[
                {
                    "dataset_path": dataset_path,
                    "episode_index": episode_index,
                    "field": pose_field,
                    "publish_rate": publish_rate,
                    "speed": speed,
                    "loop": True,
                    "frame_id": "base_link",
                    "path_stride": 5,
                }
            ],
        ),
    ]

    if enable_rviz:
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        )

    return nodes


def generate_launch_description():
    robot_package = Path(get_package_share_directory("r1_description"))
    dataset_path = os.environ.get("LEROBOT_DATASET_PATH", "")
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dataset_path",
                default_value=dataset_path,
                description="LeRobot v3 dataset directory; set LEROBOT_DATASET_PATH or pass this argument.",
            ),
            DeclareLaunchArgument(
                "urdf_path",
                default_value=str(robot_package / "urdf/r1_fixed.urdf"),
                description="R1 fixed URDF from the canonical r1_description source package.",
            ),
            DeclareLaunchArgument(
                "mesh_root",
                default_value=str(robot_package / "meshes"),
                description="Canonical r1_description mesh directory used by RViz.",
            ),
            DeclareLaunchArgument("episode_index", default_value="0"),
            DeclareLaunchArgument("field", default_value="action"),
            DeclareLaunchArgument("pose_field", default_value="action.pose"),
            DeclareLaunchArgument("speed", default_value="1.0"),
            DeclareLaunchArgument("publish_rate", default_value="30.0"),
            DeclareLaunchArgument("enable_rviz", default_value="true"),
            OpaqueFunction(function=launch_setup),
        ]
    )
