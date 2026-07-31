#!/usr/bin/python3
"""
Publish complete R1 joint states, using Isaac telemetry when available.

Local documentation referenced:
- /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/api/init_shutdown.md
- /home/gtk/ai_docs/docs.ros.org/en/rolling/p/rclpy/rclpy.node.md

关键修复点：
1. /isaac_joint_states -> /joint_states 时，严格按 joint name 映射 position/velocity/effort。
2. 不依赖 Isaac Sim 发布的数组顺序。
3. DEFAULT_JOINTS 只负责补齐缺失关节，不覆盖 Isaac 已发布的真实手臂/夹爪状态。
4. shebang 固定为 /usr/bin/python3，避免 conda Python 污染 ROS Humble rclpy。
"""

import math
from collections import OrderedDict

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


# 最终 /joint_states 的稳定输出顺序。
# 这些关节如果 Isaac 没发布，则用默认值补齐；
# 如果 Isaac 发布了，则必须使用 Isaac 的实时值。
DEFAULT_JOINTS = OrderedDict([
    ("waist_joint", 0.0),
    ("body_joint", 0.0),

    ("wheel_L_joint", 0.0),
    ("wheel_R_joint", 0.0),
    ("caster_LF_joint", 0.0),
    ("caster_RF_joint", 0.0),
    ("caster_LB_joint", 0.0),
    ("caster_RB_joint", 0.0),

    ("left_joint1", 0.0),
    ("left_joint2", 0.0),
    ("left_joint3", 0.0),
    ("left_joint4", 0.0),
    ("left_joint5", 0.0),
    ("left_joint6", 0.0),
    ("left_joint7", 0.0),

    ("right_joint1", 0.0),
    ("right_joint2", 0.0),
    ("right_joint3", 0.0),
    ("right_joint4", 0.0),
    ("right_joint5", 0.0),
    ("right_joint6", 0.0),
    ("right_joint7", 0.0),

    ("left_PGIA_joint1", 0.02),
    ("left_PGIA_joint2", 0.02),
    ("right_PGIA_joint1", 0.02),
    ("right_PGIA_joint2", 0.02),
])


class JointStateFill(Node):
    def __init__(self):
        super().__init__("joint_state_fill")

        self.declare_parameter("input_topic", "/isaac_joint_states")
        self.declare_parameter("output_topic", "/joint_states")
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("debug_mapping", False)

        self.input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().double_value
        )
        self.debug_mapping = (
            self.get_parameter("debug_mapping").get_parameter_value().bool_value
        )

        # 按输出顺序保存最新状态；DEFAULT_JOINTS 只提供初始/缺省值。
        self.joint_positions = OrderedDict(DEFAULT_JOINTS)
        self.joint_velocities = {name: 0.0 for name in DEFAULT_JOINTS}
        self.joint_efforts = {name: 0.0 for name in DEFAULT_JOINTS}

        self.latest_stamp = None
        self.received_isaac_state = False
        self._warned_missing_default_joints = False
        self._debug_counter = 0

        self.publisher = self.create_publisher(JointState, self.output_topic, 10)
        self.subscription = self.create_subscription(
            JointState,
            self.input_topic,
            self._joint_state_cb,
            10,
        )
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_filled)

        self.get_logger().info(
            f"Filling joint states: {self.input_topic} -> {self.output_topic}"
        )
        self.get_logger().info(
            "Mapping mode: by joint name, not by array index"
        )
        self.get_logger().info(
            f"Default joint count: {len(DEFAULT_JOINTS)}, publish_rate={self.publish_rate:.1f}Hz"
        )

    @staticmethod
    def _safe_array_get(array, index, default=0.0):
        if array is None:
            return default
        if index >= len(array):
            return default
        try:
            value = float(array[index])
            if math.isnan(value) or math.isinf(value):
                return default
            return value
        except Exception:
            return default

    def _joint_state_cb(self, msg: JointState):
        self.latest_stamp = msg.header.stamp
        self.received_isaac_state = True

        # 1. 先把 Isaac 输入按 name 建成映射。
        #    这是关键：不能假设 Isaac 的 name 顺序和 DEFAULT_JOINTS 一致。
        src_pos = {}
        src_vel = {}
        src_eff = {}

        for index, joint_name in enumerate(msg.name):
            src_pos[joint_name] = self._safe_array_get(msg.position, index, 0.0)
            src_vel[joint_name] = self._safe_array_get(msg.velocity, index, 0.0)
            src_eff[joint_name] = self._safe_array_get(msg.effort, index, 0.0)

        # 2. DEFAULT_JOINTS 中已有的关节：如果 Isaac 发布了，就用 Isaac 实时值覆盖。
        for joint_name in DEFAULT_JOINTS:
            if joint_name in src_pos:
                self.joint_positions[joint_name] = src_pos[joint_name]
                self.joint_velocities[joint_name] = src_vel.get(joint_name, 0.0)
                self.joint_efforts[joint_name] = src_eff.get(joint_name, 0.0)

        # 3. Isaac 发布但 DEFAULT_JOINTS 没列出的关节：追加保留，避免误删其他非 fixed joint。
        for joint_name in msg.name:
            if joint_name not in self.joint_positions:
                self.joint_positions[joint_name] = src_pos.get(joint_name, 0.0)
                self.joint_velocities[joint_name] = src_vel.get(joint_name, 0.0)
                self.joint_efforts[joint_name] = src_eff.get(joint_name, 0.0)

        missing_defaults = [
            joint_name
            for joint_name in DEFAULT_JOINTS
            if joint_name not in src_pos
        ]
        if missing_defaults and not self._warned_missing_default_joints:
            self.get_logger().warn(
                "Isaac joint state is missing default joints; keeping defaults for: "
                + ", ".join(missing_defaults)
            )
            self._warned_missing_default_joints = True

        if self.debug_mapping:
            self._debug_counter += 1
            if self._debug_counter % 100 == 1:
                watch = [
                    "left_joint1", "left_joint2", "left_joint3",
                    "right_joint1", "right_joint2", "right_joint3",
                    "left_PGIA_joint1", "right_PGIA_joint1",
                ]
                parts = []
                for name in watch:
                    parts.append(
                        f"{name}: isaac={src_pos.get(name, None)} -> out={self.joint_positions.get(name, None)}"
                    )
                self.get_logger().info(" | ".join(parts))

    def _publish_filled(self):
        msg = JointState()

        if (
            self.latest_stamp is not None
            and (self.latest_stamp.sec != 0 or self.latest_stamp.nanosec != 0)
        ):
            msg.header.stamp = self.latest_stamp
        else:
            msg.header.stamp = self.get_clock().now().to_msg()

        # 关键：name/position/velocity/effort 在同一个循环中 append，
        # 保证 name[i] 一定对应 position[i]。
        for joint_name, position in self.joint_positions.items():
            msg.name.append(joint_name)
            msg.position.append(float(position))
            msg.velocity.append(float(self.joint_velocities.get(joint_name, 0.0)))
            msg.effort.append(float(self.joint_efforts.get(joint_name, 0.0)))

        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = JointStateFill()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
