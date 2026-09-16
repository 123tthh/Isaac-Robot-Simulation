# Reference: https://docs.ros.org/en/rolling/Tutorials/Intermediate/RViz/RViz-User-Guide/RViz-User-Guide.md
# Reference: https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Creating-Launch-Files.md

from __future__ import annotations

import math
import os

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.node import Node

from r1_lerobot_sim.trajectory_io import load_pose_trajectory


class PosePathReplay(Node):
    def __init__(self) -> None:
        super().__init__("r1_lerobot_pose_path_replay")
        self.declare_parameter(
            "dataset_path", os.environ.get("LEROBOT_DATASET_PATH", "")
        )
        self.declare_parameter("episode_index", 0)
        self.declare_parameter("field", "action.pose")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("path_stride", 5)

        dataset_path = self.get_parameter("dataset_path").value
        self.episode_index = int(self.get_parameter("episode_index").value)
        self.field = str(self.get_parameter("field").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)
        self.speed = max(1.0e-6, float(self.get_parameter("speed").value))
        self.loop = bool(self.get_parameter("loop").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.path_stride = max(1, int(self.get_parameter("path_stride").value))

        self.trajectory = load_pose_trajectory(dataset_path, self.episode_index, self.field)
        self.left_pose_pub = self.create_publisher(PoseStamped, "/lerobot/left_target_pose", 10)
        self.right_pose_pub = self.create_publisher(PoseStamped, "/lerobot/right_target_pose", 10)
        self.left_path_pub = self.create_publisher(Path, "/lerobot/left_target_path", 1)
        self.right_path_pub = self.create_publisher(Path, "/lerobot/right_target_path", 1)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish)
        self.start_time = self.get_clock().now()
        self._published_paths = False

        duration = float(self.trajectory.timestamps[-1] - self.trajectory.timestamps[0])
        self.get_logger().info(
            f"Loaded pose field {self.field}: episode={self.episode_index}, "
            f"frames={len(self.trajectory.timestamps)}, duration={duration:.2f}s"
        )

    def _publish(self) -> None:
        now = self.get_clock().now()
        if not self._published_paths:
            self.left_path_pub.publish(self._build_path(now, left=True))
            self.right_path_pub.publish(self._build_path(now, left=False))
            self._published_paths = True

        elapsed = (now - self.start_time).nanoseconds * 1.0e-9 * self.speed
        t0 = float(self.trajectory.timestamps[0])
        t1 = float(self.trajectory.timestamps[-1])
        duration = max(1.0e-9, t1 - t0)
        target_time = t0 + math.fmod(elapsed, duration) if self.loop else min(t1, t0 + elapsed)
        index = int(self.trajectory.timestamps.searchsorted(target_time, side="left"))
        index = max(0, min(index, len(self.trajectory.timestamps) - 1))

        self.left_pose_pub.publish(self._pose_msg(now, self.trajectory.poses[index], left=True))
        self.right_pose_pub.publish(self._pose_msg(now, self.trajectory.poses[index], left=False))

    def _build_path(self, stamp, left: bool) -> Path:
        msg = Path()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        for row in self.trajectory.poses[:: self.path_stride]:
            msg.poses.append(self._pose_msg(stamp, row, left))
        return msg

    def _pose_msg(self, stamp, row, left: bool) -> PoseStamped:
        base = 0 if left else 8
        msg = PoseStamped()
        msg.header.stamp = stamp.to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(row[base + 0])
        msg.pose.position.y = float(row[base + 1])
        msg.pose.position.z = float(row[base + 2])
        msg.pose.orientation.x = float(row[base + 3])
        msg.pose.orientation.y = float(row[base + 4])
        msg.pose.orientation.z = float(row[base + 5])
        msg.pose.orientation.w = float(row[base + 6])
        return msg


def main() -> None:
    rclpy.init()
    node = PosePathReplay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
