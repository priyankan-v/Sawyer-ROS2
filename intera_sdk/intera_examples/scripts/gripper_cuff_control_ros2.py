#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

import intera_interface
from intera_interface.robot_params_ros2 import RobotParamsROS2


class GripperConnectROS2:
    """Connect cuff button presses to gripper open/close commands."""

    def __init__(self, node, arm, lights=True):
        self._node = node
        self._arm = arm
        self._cuff = intera_interface.CuffROS2(limb=arm, node=node)
        self._lights = intera_interface.LightsROS2(node=node) if lights else None
        self._gripper = None

        if self._lights:
            self._cuff.register_callback(self._light_action, f"{arm}_cuff")

        try:
            self._gripper = intera_interface.GripperROS2(f"{arm}_gripper", node=node)
            if not self._gripper.is_calibrated():
                self._gripper.calibrate()

            self._cuff.register_callback(self._close_action, f"{arm}_button_upper")
            self._cuff.register_callback(self._open_action, f"{arm}_button_lower")

            self._node.get_logger().info(f"{self._gripper.name} cuff control initialized")
        except Exception as exc:
            self._gripper = None
            self._node.get_logger().warning(
                f"{arm.capitalize()} gripper is not connected. Running cuff-light only. ({exc})"
            )

    def _open_action(self, value):
        if value and self._gripper and self._gripper.is_ready():
            self._node.get_logger().debug("gripper open triggered")
            self._gripper.open()
            if self._lights:
                self._set_lights("red", False)
                self._set_lights("green", True)

    def _close_action(self, value):
        if value and self._gripper and self._gripper.is_ready():
            self._node.get_logger().debug("gripper close triggered")
            self._gripper.close()
            if self._lights:
                self._set_lights("green", False)
                self._set_lights("red", True)

    def _light_action(self, value):
        if value:
            self._node.get_logger().debug("cuff grasp triggered")
        else:
            self._node.get_logger().debug("cuff release triggered")
        if self._lights:
            self._set_lights("red", False)
            self._set_lights("green", False)
            self._set_lights("blue", value)

    def _set_lights(self, color, value):
        self._lights.set_light_state(f"head_{color}_light", on=bool(value))
        self._lights.set_light_state(f"{self._arm}_hand_{color}_light", on=bool(value))


def main():
    rclpy.init()
    node = Node("sdk_gripper_cuff_control_ros2")

    rp = RobotParamsROS2(node)
    valid_limbs = rp.get_limb_names() or ["right"]
    choices = list(valid_limbs)
    if len(valid_limbs) > 1:
        choices = choices + ["all_limbs"]

    parser = argparse.ArgumentParser(description="SDK Gripper Button Control Example (ROS2)")
    parser.add_argument(
        "-g",
        "--gripper",
        dest="gripper",
        default=choices[0],
        choices=choices,
        help="gripper limb to control",
    )
    parser.add_argument(
        "-n",
        "--no-lights",
        dest="lights",
        action="store_false",
        help="do not trigger lights on cuff grasp",
    )
    args = parser.parse_args()

    arms = (args.gripper,) if args.gripper != "all_limbs" else tuple(valid_limbs)
    _grip_ctrls = [GripperConnectROS2(node, arm, args.lights) for arm in arms]

    node.get_logger().info("Press cuff buttons for gripper control. Spinning...")
    try:
        while node.context.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
