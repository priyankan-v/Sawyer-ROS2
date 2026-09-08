#!/usr/bin/env python3

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node

try:
    import xacro_jade as xacro
except ImportError:
    import xacro

from intera_core_msgs.msg import URDFConfiguration


def xacro_parse(filename):
    if hasattr(xacro, "process_file"):
        doc = xacro.process_file(filename)
        return doc.toprettyxml(indent="  ")
    doc = xacro.parse(None, filename)
    xacro.process_doc(doc, in_order=True)
    return doc.toprettyxml(indent="  ")


def send_urdf(node, parent_link, root_joint, urdf_filename, duration):
    pub = node.create_publisher(URDFConfiguration, "/robot/urdf", 10)
    msg = URDFConfiguration()
    msg.time = node.get_clock().now().to_msg()
    msg.link = parent_link
    msg.joint = root_joint
    msg.urdf = xacro_parse(urdf_filename)

    end_time = time.monotonic() + duration
    while node.context.ok() and time.monotonic() < end_time:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=0.2)


def main():
    parser = argparse.ArgumentParser(description="Send URDF fragment to robot (ROS2)")
    parser.add_argument("-f", "--file", metavar="PATH", required=True, help="path to URDF file to send")
    parser.add_argument("-l", "--link", default="right_hand", help="URDF link to attach fragment")
    parser.add_argument("-j", "--joint", default="right_gripper_base", help="root joint in fragment")
    parser.add_argument("-d", "--duration", type=lambda t: abs(float(t)), default=5.0)
    args = parser.parse_args()

    if not os.access(args.file, os.R_OK):
        print(f"Cannot read file at '{args.file}'")
        return 1

    rclpy.init()
    node = Node("rsdk_configure_urdf_ros2")
    try:
        send_urdf(node, args.link, args.joint, args.file, args.duration)
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
