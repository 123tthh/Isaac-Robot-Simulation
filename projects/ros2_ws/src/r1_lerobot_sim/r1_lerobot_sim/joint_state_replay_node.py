# Reference: https://docs.ros.org/en/rolling/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher-py.md
# Reference: https://docs.ros.org/en/rolling/p/tf2_ros_py/tf2_ros.transform_broadcaster.md

from __future__ import annotations

import math
import os

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from r1_lerobot_sim.trajectory_io import (
    load_joint_trajectory,
    map_gripper_to_pgia,
    read_urdf_movable_joints,
)


class JointStateReplay(Node):
    def __init__(self) -> None:
        super().__init__("r1_lerobot_joint_state_replay")
        robot_package = get_package_share_directory("r1_description")
        self.declare_parameter(
            "dataset_path", os.environ.get("LEROBOT_DATASET_PATH", "")
        )
        self.declare_parameter("npz_path", "")
        self.declare_parameter("urdf_path", f"{robot_package}/urdf/r1_fixed.urdf")
        self.declare_parameter("episode_index", 0)
        self.declare_parameter("field", "action")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("gripper_mode", "episode_minmax")
        self.declare_parameter("gripper_closed", 0.04)
        self.declare_parameter("gripper_open", 0.12)
        self.declare_parameter("output_topic", "/joint_states")

        dataset_path = self.get_parameter("dataset_path").value
        npz_path = self.get_parameter("npz_path").value or None
        self.urdf_path = self.get_parameter("urdf_path").value
        self.episode_index = int(self.get_parameter("episode_index").value)
        self.field = str(self.get_parameter("field").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.speed = max(1.0e-6, float(self.get_parameter("speed").value))
        self.loop = bool(self.get_parameter("loop").value)
        self.gripper_mode = str(self.get_parameter("gripper_mode").value)
        self.gripper_closed = float(self.get_parameter("gripper_closed").value)
        self.gripper_open = float(self.get_parameter("gripper_open").value)
        output_topic = str(self.get_parameter("output_topic").value)

        self.trajectory = load_joint_trajectory(dataset_path, self.episode_index, self.field, npz_path)
        self.gripper_min = None
        self.gripper_max = None
        if self.trajectory.gripper is not None:
            self.gripper_min = self.trajectory.gripper.min(axis=0)
            self.gripper_max = self.trajectory.gripper.max(axis=0)
        self.output_joint_names, self.default_positions = read_urdf_movable_joints(self.urdf_path)
        self.publisher = self.create_publisher(JointState, output_topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        self.start_time = self.get_clock().now()
        self.last_index = -1

        duration = float(self.trajectory.timestamps[-1] - self.trajectory.timestamps[0])
        self.get_logger().info(
            f"Loaded episode {self.episode_index}: {len(self.trajectory.timestamps)} frames, "
            f"{duration:.2f}s, field={self.field}, output={output_topic}"
        )
        self.get_logger().info(f"URDF movable joints: {len(self.output_joint_names)} from {self.urdf_path}")
        if self.trajectory.gripper is not None:
            self.get_logger().info(
                "Gripper mapping mode="
                f"{self.gripper_mode}, left range=[{self.gripper_min[0]:.4f}, {self.gripper_max[0]:.4f}], "
                f"right range=[{self.gripper_min[1]:.4f}, {self.gripper_max[1]:.4f}]"
            )

    def _publish(self) -> None:
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds * 1.0e-9 * self.speed
        t0 = float(self.trajectory.timestamps[0])
        t1 = float(self.trajectory.timestamps[-1])
        duration = max(1.0e-9, t1 - t0)
        if self.loop:
            target_time = t0 + math.fmod(elapsed, duration)
        else:
            target_time = min(t1, t0 + elapsed)
        index = int(self.trajectory.timestamps.searchsorted(target_time, side="left"))
        index = max(0, min(index, len(self.trajectory.timestamps) - 1))
        self.last_index = index

        positions = dict(self.default_positions)
        for joint_name, value in zip(self.trajectory.joint_names, self.trajectory.positions[index]):
            positions[joint_name] = float(value)

        if self.trajectory.gripper is not None:
            left_value, right_value = self.trajectory.gripper[index]
            positions["left_PGIA_joint1"] = map_gripper_to_pgia(
                left_value,
                self.gripper_mode,
                self.gripper_closed,
                self.gripper_open,
                self.gripper_min[0],
                self.gripper_max[0],
            )
            positions["left_PGIA_joint2"] = positions["left_PGIA_joint1"]
            positions["right_PGIA_joint1"] = map_gripper_to_pgia(
                right_value,
                self.gripper_mode,
                self.gripper_closed,
                self.gripper_open,
                self.gripper_min[1],
                self.gripper_max[1],
            )
            positions["right_PGIA_joint2"] = positions["right_PGIA_joint1"]

        msg = JointState()
        msg.header.stamp = now.to_msg()
        msg.name = list(self.output_joint_names)
        msg.position = [float(positions.get(name, 0.0)) for name in msg.name]
        msg.velocity = [0.0] * len(msg.name)
        msg.effort = [0.0] * len(msg.name)
        self.publisher.publish(msg)


def main() -> None:
    rclpy.init()
    node = JointStateReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
