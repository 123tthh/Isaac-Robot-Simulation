#!/usr/bin/env python3
"""
将 /odom (nav_msgs/Odometry) 转为 TF 广播: odom → base_link
同时发布 world → odom 静态变换

Local documentation referenced:
- https://docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.md
- https://docs.ros.org/en/rolling/p/tf2_ros_py/tf2_ros.transform_broadcaster.md

修复说明:
  use_sim_time=True 时，节点启动初期 /clock 尚未收到，
  get_clock().now() 返回 Time(sec=0)。
  如果此时立即发布 world→odom 静态 TF（时间戳=0），
  而 odom→base_link 的动态 TF 时间戳来自 /odom（比如 sec=1164），
  tf2_buffer 会判定两者不属于同一棵树。

  修复方法：等收到第一条 /odom 消息后，
  用 /odom 消息的时间戳发布 world→odom 静态 TF，
  确保所有 TF 时间戳同源。
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomToTf(Node):

    def __init__(self):
        super().__init__("odom_to_tf")
        self.br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)
        self._static_sent = False

        self.sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10
        )
        self.get_logger().info(
            "odom_to_tf ready: waiting for first /odom to publish world→odom static TF"
        )

    def _odom_cb(self, msg: Odometry):
        # 第一次收到 /odom 时，用其时间戳发布 world→odom 静态 TF
        # 此后每次收到 /odom 都发布动态 odom→base_link TF
        if not self._static_sent:
            static_t = TransformStamped()
            static_t.header.stamp = msg.header.stamp  # 与 /odom 同一时间戳
            static_t.header.frame_id = "world"
            static_t.child_frame_id = "odom"
            static_t.transform.rotation.w = 1.0
            self.static_br.sendTransform(static_t)
            self._static_sent = True
            self.get_logger().info(
                f"Published static world→odom TF at sim_time={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"
            )

        # 动态 TF: odom → base_link
        t = TransformStamped()
        t.header.stamp = msg.header.stamp   # 直接用 /odom 的时间戳，不用 get_clock()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
