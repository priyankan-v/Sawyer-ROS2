#!/usr/bin/env python3

import argparse
import json

import errno
import rclpy
from rclpy.node import Node

from intera_interface.limb_ros2 import LimbROS2
from intera_interface.robot_enable_ros2 import RobotEnableROS2

from intera_external_devices.joystick_ros2 import (
    LogitechControllerROS2,
    PS3ControllerROS2,
    XboxControllerROS2,
)


class JointPositionJoystickROS2(Node):
    def __init__(self, joystick_type, limb, auto_enable):
        super().__init__("sdk_joint_position_joystick_ros2")
        self._limb = limb
        self._robot_enable = None
        self._limb_iface = None

        try:
            self._robot_enable = RobotEnableROS2(node=self, versioned=False)
            if auto_enable:
                state = self._robot_enable.state()
                if state is not None and not state.enabled:
                    self.get_logger().info("Auto-enable requested. Enabling robot...")
                    self._robot_enable.enable()
        except OSError as exc:
            # Keep script usable in non-hardware environments while still exercising the API when available.
            if exc.errno == errno.ETIMEDOUT:
                self.get_logger().warning("Robot state not available; continuing without RobotEnableROS2")
            else:
                self.get_logger().warning(f"RobotEnableROS2 unavailable: {exc}")

        self._limb_iface = LimbROS2(
            limb=limb,
            node=self,
            require_joint_states=True,
            require_endpoint_state=False,
            require_tip_states=False,
        )
        self._joint_names = self._limb_iface.joint_names()
        self._joint_positions = self._limb_iface.joint_angles()

        if joystick_type == "xbox":
            self._joystick = XboxControllerROS2(self)
        elif joystick_type == "logitech":
            self._joystick = LogitechControllerROS2(self)
        else:
            self._joystick = PS3ControllerROS2(self)

        self._active_joint_index = [0, 1, 2, 3]
        self._step = 0.1

        self.create_timer(0.01, self._tick)

    def _rotate_joint_selection(self):
        self._active_joint_index = [(i + 1) % len(self._joint_names) for i in self._active_joint_index]
        selected = [self._joint_names[i] for i in self._active_joint_index]
        self.get_logger().info(f"Active joints: {json.dumps(selected)}")

    def _publish_positions(self, updates):
        self._limb_iface.set_joint_positions(updates)

    def _tick(self):
        updates = {}
        self._joint_positions = self._limb_iface.joint_angles()

        axes = [
            self._joystick.stick_value("leftStickHorz"),
            self._joystick.stick_value("leftStickVert"),
            self._joystick.stick_value("rightStickHorz"),
            self._joystick.stick_value("rightStickVert"),
        ]

        for axis_idx, value in enumerate(axes):
            if abs(value) < 1e-6:
                continue
            joint_idx = self._active_joint_index[axis_idx]
            joint_name = self._joint_names[joint_idx]
            updates[joint_name] = self._joint_positions[joint_name] + (self._step * value)

        if self._joystick.button_down("leftBumper"):
            self._rotate_joint_selection()

        if self._joystick.button_down("function1") or self._joystick.button_down("function2"):
            self.get_logger().info("Use sticks to move 4 selected joints; left bumper cycles joint selection")

        if updates:
            self._publish_positions(updates)


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def parse_args():
    parser = argparse.ArgumentParser(description="ROS2 joystick joint position example")
    parser.add_argument("--joystick", required=True, choices=["xbox", "logitech", "ps3"])
    parser.add_argument("--limb", default="right", choices=["right", "left"])
    parser.add_argument("--auto-enable", default="false")
    return parser.parse_known_args()[0]


def main():
    args = parse_args()
    rclpy.init()
    node = JointPositionJoystickROS2(args.joystick, args.limb, _as_bool(args.auto_enable))
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
