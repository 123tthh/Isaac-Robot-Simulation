#!/usr/bin/env python3
"""
简单的夹爪测试脚本
用于验证gripper_hardware接口的基本功能
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time


class GripperTester(Node):
    def __init__(self):
        super().__init__("gripper_tester")

        # 发布者和订阅者
        self.command_sub = self.create_subscription(JointState, "/gripper_command", self.command_callback, 10)

        self.state_pub = self.create_publisher(JointState, "/gripper_state", 10)

        # 当前状态
        self.current_position = 0.0
        self.command_received = False

        # 定时器 - 发布状态
        self.create_timer(0.02, self.publish_state)  # 50Hz

        self.get_logger().info("Gripper tester started")
        self.get_logger().info("Publishing states to /gripper_state")
        self.get_logger().info("Listening for commands on /gripper_command")

    def command_callback(self, msg):
        """接收命令"""
        if msg.name and msg.position:
            self.command_received = True
            target = msg.position[0]
            self.get_logger().info(f"✅ Command received: {target:.4f} m")

    def publish_state(self):
        """发布状态"""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper_finger_joint"]
        msg.position = [self.current_position]
        msg.velocity = [0.0]
        msg.effort = [0.0]

        self.state_pub.publish(msg)

        # 慢慢移动位置
        self.current_position += 0.0001
        if self.current_position > 0.085:
            self.current_position = 0.0


def main():
    print("\n" + "=" * 60)
    print("夹爪硬件接口测试器")
    print("=" * 60)
    print("\n📋 测试步骤:")
    print("1. 此脚本将发布状态到 /gripper_state")
    print("2. 启动你的ros2_control节点（加载gripper_hardware）")
    print("3. 此脚本将监听 /gripper_command 话题")
    print("4. 发送控制命令测试")
    print("\n等待命令...\n")

    rclpy.init()
    tester = GripperTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print("\n\n测试结束")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
