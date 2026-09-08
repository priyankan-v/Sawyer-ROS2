#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

import intera_interface


def echo_input(node, nav_name="right"):
    nav = intera_interface.NavigatorROS2(node=node)

    def back_pressed(v):
        node.get_logger().info(f"Button 'Back': {nav.button_string_lookup(v)}")

    def rethink_pressed(v):
        node.get_logger().info(f"Button 'Rethink': {nav.button_string_lookup(v)}")

    def circle_pressed(v):
        node.get_logger().info(f"Button 'Circle': {nav.button_string_lookup(v)}")

    def square_pressed(v):
        node.get_logger().info(f"Button 'Square': {nav.button_string_lookup(v)}")

    def x_pressed(v):
        node.get_logger().info(f"Button 'X': {nav.button_string_lookup(v)}")

    def ok_pressed(v):
        node.get_logger().info(f"Button 'OK': {nav.button_string_lookup(v)}")

    def wheel_moved(v):
        node.get_logger().info(f"Wheel value: {v}")

    nav.register_callback(back_pressed, "_".join([nav_name, "button_back"]))
    nav.register_callback(rethink_pressed, "_".join([nav_name, "button_show"]))
    nav.register_callback(circle_pressed, "_".join([nav_name, "button_circle"]))
    nav.register_callback(square_pressed, "_".join([nav_name, "button_square"]))
    nav.register_callback(x_pressed, "_".join([nav_name, "button_triangle"]))
    nav.register_callback(ok_pressed, "_".join([nav_name, "button_ok"]))
    nav.register_callback(wheel_moved, "_".join([nav_name, "wheel"]))

    node.get_logger().info("Press input buttons on the navigator; input will be echoed here.")

    end_time = time.monotonic() + 10.0
    while node.context.ok() and time.monotonic() < end_time:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    parser = argparse.ArgumentParser(description="SDK Navigator Example (ROS2)")
    parser.add_argument(
        "-n",
        "--navigator",
        dest="nav_name",
        default="right",
        choices=["right", "head"],
        help="Navigator on which to run example",
    )
    args = parser.parse_args()

    rclpy.init()
    node = Node("sdk_navigator_ros2")
    try:
        echo_input(node, args.nav_name)
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
