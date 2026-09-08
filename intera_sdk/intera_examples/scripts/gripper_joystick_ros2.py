#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

from intera_io.io_interface_ros2 import IODeviceInterfaceROS2

from intera_external_devices.joystick_ros2 import (
    LogitechControllerROS2,
    PS3ControllerROS2,
    XboxControllerROS2,
)


class GripperJoystickROS2(Node):
    MAX_POSITION = 0.041667
    MIN_POSITION = 0.0
    MAX_VELOCITY = 3.0
    MIN_VELOCITY = 0.15

    def __init__(self, joystick_type, limb):
        super().__init__("sdk_gripper_joystick_ros2")
        self._limb = limb
        self._gripper_name = f"{limb}_gripper"
        self._position = self.MAX_POSITION
        self._velocity = 1.0
        self._io = IODeviceInterfaceROS2(
            node=self,
            node_name="end_effector",
            dev_name=self._gripper_name,
            require_config=False,
            require_state=False,
        )

        if joystick_type == "xbox":
            self._joystick = XboxControllerROS2(self)
        elif joystick_type == "logitech":
            self._joystick = LogitechControllerROS2(self)
        else:
            self._joystick = PS3ControllerROS2(self)

        self._position_increment = (self.MAX_POSITION - self.MIN_POSITION) / 8.0
        self._velocity_increment = (self.MAX_VELOCITY - self.MIN_VELOCITY) / 8.0

        self.create_timer(0.01, self._tick)

    def _publish_signal_set(self, signal_name, signal_type, value):
        self._io.set_signal_value(signal_name, value, signal_type=signal_type)

    def _set_position(self, target):
        self._position = max(self.MIN_POSITION, min(self.MAX_POSITION, target))
        self._publish_signal_set("position_m", "float", self._position)
        self.get_logger().info(f"gripper position_m={self._position:.5f}")

    def _set_velocity(self, target):
        self._velocity = max(self.MIN_VELOCITY, min(self.MAX_VELOCITY, target))
        self._publish_signal_set("speed_mps", "float", self._velocity)
        self.get_logger().info(f"gripper speed_mps={self._velocity:.3f}")

    def _tick(self):
        if self._joystick.button_down("btnUp"):
            self._publish_signal_set("calibrate", "bool", True)
            self.get_logger().info("gripper calibrate")

        if self._joystick.button_down("btnLeft"):
            self._publish_signal_set("reboot", "bool", True)
            self.get_logger().info("gripper reboot")

        if self._joystick.button_down("leftTrigger"):
            self._set_position(self.MIN_POSITION)

        if self._joystick.button_up("leftTrigger"):
            self._set_position(self.MAX_POSITION)

        if self._joystick.button_down("leftBumper"):
            self._publish_signal_set("go", "bool", False)
            self.get_logger().info("gripper stop")

        pos_axis = self._joystick.stick_value("leftStickVert")
        if abs(pos_axis) > 1e-6:
            self._set_position(self._position + pos_axis * self._position_increment)

        vel_axis = self._joystick.stick_value("rightStickVert")
        if abs(vel_axis) > 1e-6:
            self._set_velocity(self._velocity + vel_axis * self._velocity_increment)

        if self._joystick.button_down("function1") or self._joystick.button_down("function2"):
            self.get_logger().info("left trigger close/open, left stick position, right stick velocity")


def parse_args():
    parser = argparse.ArgumentParser(description="ROS2 joystick gripper example")
    parser.add_argument("--joystick", required=True, choices=["xbox", "logitech", "ps3"])
    parser.add_argument("--limb", default="right", choices=["right", "left"])
    return parser.parse_known_args()[0]


def main():
    args = parse_args()
    rclpy.init()
    node = GripperJoystickROS2(args.joystick, args.limb)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
