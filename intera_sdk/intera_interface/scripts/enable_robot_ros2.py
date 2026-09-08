#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node

import intera_interface


def main():
    parser = argparse.ArgumentParser(description="Enable/disable/reset/stop robot (ROS2)")
    parser.add_argument("-s", "--state", const="state", dest="actions", action="append_const", help="print current robot state")
    parser.add_argument("-e", "--enable", const="enable", dest="actions", action="append_const", help="enable the robot")
    parser.add_argument("-d", "--disable", const="disable", dest="actions", action="append_const", help="disable the robot")
    parser.add_argument("-r", "--reset", const="reset", dest="actions", action="append_const", help="reset the robot")
    parser.add_argument("-S", "--stop", const="stop", dest="actions", action="append_const", help="stop the robot")
    args = parser.parse_args()

    if args.actions is None:
        parser.print_usage()
        print("No action defined")
        return 0

    rclpy.init()
    node = Node("sdk_robot_enable_ros2")
    try:
        rs = intera_interface.RobotEnableROS2(node=node, versioned=False)
        for act in args.actions:
            if act == "state":
                print(rs.state())
            elif act == "enable":
                rs.enable()
            elif act == "disable":
                rs.disable()
            elif act == "reset":
                rs.reset()
            elif act == "stop":
                rs.stop()
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
