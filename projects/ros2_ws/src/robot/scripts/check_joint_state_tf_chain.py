#!/usr/bin/env python3
"""Diagnose the Isaac joint-state -> ROS joint-state -> TF chain for R1."""

import sys
from typing import Iterable

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


REQUIRED_JOINTS = [
    "waist_joint",
    "body_joint",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "left_joint7",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "right_joint7",
    "left_PGIA_joint1",
    "left_PGIA_joint2",
    "right_PGIA_joint1",
    "right_PGIA_joint2",
]

TF_CHECKS = [
    ("base_link", "body_link"),
    ("base_link", "arm_attach_L_link"),
    ("base_link", "arm_attach_R_link"),
    ("base_link", "left_Link7"),
    ("base_link", "right_Link7"),
]


class JointStateTfChainChecker(Node):
    def __init__(self):
        super().__init__("check_joint_state_tf_chain")
        self.declare_parameter("timeout_sec", 5.0)
        self.timeout_sec = self.get_parameter("timeout_sec").value

        self.isaac_msg = None
        self.joint_msg = None
        self.create_subscription(
            JointState, "/isaac_joint_states", self._isaac_cb, 10
        )
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _isaac_cb(self, msg):
        self.isaac_msg = msg

    def _joint_cb(self, msg):
        self.joint_msg = msg

    def run_checks(self):
        deadline = self.get_clock().now() + Duration(seconds=self.timeout_sec)
        while rclpy.ok() and self.get_clock().now() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.isaac_msg is not None and self.joint_msg is not None:
                break

        ok = True
        ok &= self._check_publishers("/isaac_joint_states", min_count=1)
        ok &= self._check_publishers(
            "/joint_states", expected=1, expected_node="joint_state_fill"
        )
        ok &= self._check_joint_msg("/isaac_joint_states", self.isaac_msg)
        ok &= self._check_joint_msg("/joint_states", self.joint_msg)
        ok &= self._check_transforms()
        return ok

    def _check_publishers(
        self, topic, expected=None, expected_node=None, min_count=None
    ):
        publishers = self.get_publishers_info_by_topic(topic)
        names = [info.node_name for info in publishers]
        status = "OK"
        ok = True
        if min_count is not None and len(publishers) < min_count:
            status = "ERROR"
            ok = False
        if expected is not None and len(publishers) != expected:
            status = "ERROR"
            ok = False
        if expected_node is not None and expected_node not in names:
            status = "ERROR"
            ok = False
        self.get_logger().info(
            f"[{status}] {topic} publishers={len(publishers)} nodes={names}"
        )
        return ok

    def _check_joint_msg(self, topic, msg):
        if msg is None:
            self.get_logger().error(f"[ERROR] no message received from {topic}")
            return False

        ok = True
        names = set(msg.name)
        missing = [joint for joint in REQUIRED_JOINTS if joint not in names]
        if missing:
            self.get_logger().error(f"[ERROR] {topic} missing joints: {missing}")
            ok = False
        else:
            self.get_logger().info(f"[OK] {topic} contains all required joints")

        ok &= self._check_array_length(topic, "position", msg.position, len(msg.name))
        ok &= self._check_array_length(topic, "velocity", msg.velocity, len(msg.name))
        ok &= self._check_array_length(topic, "effort", msg.effort, len(msg.name))
        return ok

    def _check_array_length(self, topic, label, values: Iterable[float], name_count):
        values = list(values)
        if values and len(values) != name_count:
            self.get_logger().error(
                f"[ERROR] {topic} {label} length={len(values)} name length={name_count}"
            )
            return False
        return True

    def _check_transforms(self):
        ok = True
        deadline = self.get_clock().now() + Duration(seconds=self.timeout_sec)
        for target, source in TF_CHECKS:
            found = False
            last_error = ""
            while rclpy.ok() and self.get_clock().now() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                try:
                    self.tf_buffer.lookup_transform(target, source, Time())
                    found = True
                    break
                except TransformException as exc:
                    last_error = str(exc)
            if found:
                self.get_logger().info(f"[OK] TF {target} -> {source}")
            else:
                self.get_logger().error(
                    f"[ERROR] missing TF {target} -> {source}: {last_error}"
                )
                ok = False
        return ok


def main():
    rclpy.init()
    node = JointStateTfChainChecker()
    try:
        ok = node.run_checks()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
