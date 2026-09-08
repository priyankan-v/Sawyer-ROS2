#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.node import Node

from intera_interface import LightsROS2


def test_light_interface(node, light_name="head_green_light"):
    lights = LightsROS2(node=node)
    available = lights.list_all_lights()
    node.get_logger().info(f"All available lights on this robot: {', '.join(available)}")

    if light_name not in available:
        node.get_logger().error(f"Light '{light_name}' not found in available lights")
        return 1

    node.get_logger().info(f"Blinking Light: {light_name}")
    initial_state = bool(lights.get_light_state(light_name))

    def on_off(name):
        return "ON" if lights.get_light_state(name) else "OFF"

    node.get_logger().info(f"Initial state: {on_off(light_name)}")
    state = not initial_state
    for _ in range(3):
        lights.set_light_state(light_name, state)
        end_time = time.monotonic() + 1.0
        while node.context.ok() and time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.05)
        node.get_logger().info(f"New state: {on_off(light_name)}")
        state = not state

    lights.set_light_state(light_name, initial_state)
    end_time = time.monotonic() + 1.0
    while node.context.ok() and time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.05)
    node.get_logger().info(f"Final state: {on_off(light_name)}")
    return 0


def main():
    parser = argparse.ArgumentParser(description="Intera SDK Lights Example: Blink (ROS2)")
    parser.add_argument(
        "-l",
        "--light_name",
        dest="light_name",
        default="head_green_light",
        help="name of Light component to use (default: head_green_light)",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("sdk_lights_blink_ros2")
    try:
        return test_light_interface(node, args.light_name)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
