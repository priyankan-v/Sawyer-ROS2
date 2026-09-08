#!/usr/bin/env python3

import argparse

import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError

import rclpy
from rclpy.node import Node

from intera_interface.camera_ros2 import CamerasROS2
from intera_interface.robot_params_ros2 import RobotParamsROS2


def show_image_callback(img_data, callback_args):
    edge_detection, window_name = callback_args
    bridge = CvBridge()
    try:
        cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")
    except CvBridgeError as err:
        print(err)
        return

    if edge_detection:
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (3, 3), 0)
        cv_image = np.hstack([cv2.Canny(blurred, 10, 100)])

    edge_str = "(Edge Detection)" if edge_detection else ""
    cv_win_name = " ".join([window_name, edge_str])
    cv2.namedWindow(cv_win_name, 0)
    cv2.imshow(cv_win_name, cv_image)
    cv2.waitKey(3)


def main():
    rclpy.init()
    node = Node("camera_display_ros2")

    rp = RobotParamsROS2(node)
    valid_cameras = rp.get_camera_names()
    if not valid_cameras:
        node.get_logger().error("Cannot detect any camera configuration parameters. Exiting.")
        node.destroy_node()
        rclpy.shutdown()
        return

    parser = argparse.ArgumentParser(description="Camera Display Example (ROS2)")
    parser.add_argument(
        "-c",
        "--camera",
        type=str,
        default="head_camera",
        choices=valid_cameras,
        help="Setup camera name for display",
    )
    parser.add_argument(
        "-r",
        "--raw",
        action="store_true",
        help="Use the raw image (unrectified) topic",
    )
    parser.add_argument(
        "-e",
        "--edge",
        action="store_true",
        help="Stream the Canny edge detection image",
    )
    parser.add_argument(
        "-g",
        "--gain",
        type=int,
        help="Set gain for camera (-1 = auto)",
    )
    parser.add_argument(
        "-x",
        "--exposure",
        type=float,
        help="Set exposure for camera (-1 = auto)",
    )
    args = parser.parse_args()

    cameras = CamerasROS2(node=node)
    if not cameras.verify_camera_exists(args.camera):
        node.get_logger().error("Could not detect the specified camera")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info(f"Opening camera '{args.camera}'...")
    cameras.start_streaming(args.camera)
    cameras.set_callback(
        args.camera,
        show_image_callback,
        rectify_image=(not args.raw),
        callback_args=(args.edge, args.camera),
    )

    if args.gain is not None:
        if cameras.set_gain(args.camera, args.gain):
            node.get_logger().info(f"Gain set to: {cameras.get_gain(args.camera)}")

    if args.exposure is not None:
        if cameras.set_exposure(args.camera, args.exposure):
            node.get_logger().info(f"Exposure set to: {cameras.get_exposure(args.camera)}")

    node.get_logger().info("camera_display_ros2 node running. Ctrl-C to quit")
    try:
        while node.context.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
