# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import errno
import re

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Empty

from intera_core_msgs.msg import RobotAssemblyState
from intera_dataflow.wait_for_ros2 import wait_for_ros2

from . import settings


class RobotEnableROS2:
    """ROS 2 control/status wrapper around robot assembly state."""

    def __init__(self, node=None, versioned=False):
        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node("intera_robot_enable_ros2")
            self._owns_node = True

        self._node = node
        self._state = None

        self._state_sub = self._node.create_subscription(
            RobotAssemblyState,
            "/robot/state",
            self._state_callback,
            10,
        )

        self._pub_enable = self._node.create_publisher(Bool, "/robot/set_super_enable", 10)
        self._pub_reset = self._node.create_publisher(Empty, "/robot/set_super_reset", 10)
        self._pub_stop = self._node.create_publisher(Empty, "/robot/set_super_stop", 10)

        wait_for_ros2(
            lambda: self._state is not None,
            node=self._node,
            timeout=10.0,
            timeout_msg="Failed to get robot state on /robot/state",
        )

        if versioned and not self.version_check():
            raise RuntimeError("Robot version compatibility check failed")

    def destroy(self):
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def _state_callback(self, msg):
        self._state = msg

    @staticmethod
    def _to_ros2_param_name(name):
        stripped = name[1:] if name.startswith("/") else name
        return stripped.replace("/", ".")

    def _get_param(self, name, default=None):
        key = self._to_ros2_param_name(name)
        if not self._node.has_parameter(key):
            self._node.get_logger().warning(f"Parameter '{key}' not declared; using default")
            return default
        param = self._node.get_parameter(key)
        return param.value

    def _toggle_enabled(self, status):
        def publish_state():
            msg = Bool()
            msg.data = bool(status)
            self._pub_enable.publish(msg)

        wait_for_ros2(
            lambda: self._state is not None and self._state.enabled == status,
            node=self._node,
            timeout=5.0,
            timeout_msg=f"Failed to {'en' if status else 'dis'}able robot",
            body=publish_state,
        )
        self._node.get_logger().info(f"Robot {'Enabled' if status else 'Disabled'}")

    def state(self):
        return self._state

    def enable(self):
        if self._state is not None and self._state.stopped:
            self._node.get_logger().info("Robot Stopped: Attempting Reset...")
            self.reset()
        self._toggle_enabled(True)

    def disable(self):
        self._toggle_enabled(False)

    def reset(self):
        error_not_stopped = "Robot is not in an error state; cannot perform reset"
        error_estop = "E-Stop is asserted; disengage E-Stop and reset the robot"

        if self._state is None:
            raise OSError(errno.EHOSTDOWN, "No robot state available")
        if not self._state.stopped:
            raise OSError(errno.EREMOTEIO, error_not_stopped)
        if self._state.estop_button == RobotAssemblyState.ESTOP_BUTTON_PRESSED:
            raise OSError(errno.EREMOTEIO, error_estop)

        self._node.get_logger().info("Resetting robot...")

        def is_reset():
            s = self._state
            return (
                s is not None
                and (not s.stopped)
                and (not s.error)
                and s.estop_button == RobotAssemblyState.ESTOP_BUTTON_UNPRESSED
                and s.estop_source == RobotAssemblyState.ESTOP_SOURCE_NONE
            )

        wait_for_ros2(
            is_reset,
            node=self._node,
            timeout=5.0,
            timeout_msg="Failed to reset robot within timeout",
            body=lambda: self._pub_reset.publish(Empty()),
        )

    def stop(self):
        wait_for_ros2(
            lambda: self._state is not None and self._state.stopped,
            node=self._node,
            timeout=5.0,
            timeout_msg="Failed to stop the robot",
            body=lambda: self._pub_stop.publish(Empty()),
        )

    def version_check(self):
        param_name = "/manifest/robot_software/version/HLR_VERSION_STRING"
        sdk_version = settings.SDK_VERSION
        robot_version = self._get_param(param_name, None)

        if not robot_version:
            self._node.get_logger().warning(
                f"RobotEnableROS2: Failed to retrieve robot version from parameters: {param_name}"
            )
            return False

        match = re.search(r"^([0-9]+)\.([0-9]+)\.([0-9]+)", str(robot_version))
        if not match:
            self._node.get_logger().warning(f"RobotEnableROS2: Invalid robot version: {robot_version}")
            return False

        normalized = f"{match.group(1)}.{match.group(2)}.{match.group(3)}"
        compatible = settings.VERSIONS_SDK2ROBOT.get(sdk_version, [])
        if normalized not in compatible:
            self._node.get_logger().error(
                f"RobotEnableROS2: Software version mismatch (robot={normalized}, sdk={sdk_version})"
            )
            return False

        return True
