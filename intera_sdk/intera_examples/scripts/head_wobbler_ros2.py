#!/usr/bin/env python3

import argparse
import random
import time

import rclpy
from rclpy.node import Node

import intera_interface


class WobblerROS2:
    def __init__(self, node):
        self._node = node
        self._done = False
        self._head = intera_interface.HeadROS2(node=node)
        self._rs = intera_interface.RobotEnableROS2(node=node, versioned=False)

        self._init_state = False
        state = self._rs.state()
        if state is not None:
            self._init_state = bool(state.enabled)

        self._node.get_logger().info("Enabling robot...")
        self._rs.enable()

    def clean_shutdown(self):
        self._node.get_logger().info("Exiting example")
        if self._done:
            self.set_neutral()

    def set_neutral(self):
        self._head.set_pan(0.0)

    def wobble(self):
        self.set_neutral()
        start = time.monotonic()
        while self._node.context.ok() and (time.monotonic() - start < 10.0):
            angle = random.uniform(-2.0, 0.95)
            while self._node.context.ok() and not (
                abs(self._head.pan() - angle) <= intera_interface.HEAD_PAN_ANGLE_TOLERANCE
            ):
                self._head.set_pan(angle, speed=0.3, timeout=0)
                rclpy.spin_once(self._node, timeout_sec=0.01)
            end_time = time.monotonic() + 1.0
            while self._node.context.ok() and time.monotonic() < end_time:
                rclpy.spin_once(self._node, timeout_sec=0.05)

        self._done = True


def main():
    parser = argparse.ArgumentParser(description="RSDK Head Example: Wobbler (ROS2)")
    parser.parse_args()

    rclpy.init()
    node = Node("rsdk_head_wobbler_ros2")
    try:
        wobbler = WobblerROS2(node)
    except OSError as exc:
        node.get_logger().error(f"Unable to start head_wobbler_ros2: {exc}")
        node.destroy_node()
        rclpy.shutdown()
        return 1
    try:
        node.get_logger().info("Wobbling...")
        wobbler.wobble()
        node.get_logger().info("Done")
    finally:
        wobbler.clean_shutdown()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
