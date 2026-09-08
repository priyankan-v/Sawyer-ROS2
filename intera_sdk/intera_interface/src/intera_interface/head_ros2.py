# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

from copy import deepcopy

import rclpy
from rclpy.node import Node

from intera_core_msgs.msg import HeadPanCommand, HeadState
from intera_dataflow.wait_for_ros2 import wait_for_ros2

from . import settings


class HeadROS2:
    """ROS 2 interface class for controlling the robot head pan."""

    def __init__(self, node=None):
        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node("intera_head_ros2")
            self._owns_node = True

        self._node = node
        self._state = {}

        self._pub_pan = self._node.create_publisher(
            HeadPanCommand,
            "/robot/head/command_head_pan",
            10,
        )

        state_topic = "/robot/head/head_state"
        self._sub_state = self._node.create_subscription(
            HeadState,
            state_topic,
            self._on_head_state,
            10,
        )

        wait_for_ros2(
            lambda: len(self._state) != 0,
            node=self._node,
            timeout=5.0,
            timeout_msg=f"Failed to get current head state from {state_topic}",
        )

    def destroy(self):
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def _on_head_state(self, msg):
        self._state["pan"] = msg.pan
        self._state["panning"] = msg.is_turning
        self._state["blocked"] = msg.is_blocked
        self._state["pan_mode"] = msg.pan_mode

    def blocked(self):
        return self._state["blocked"]

    def pan_mode(self):
        pan_mode_dict = {
            0: "PASSIVE_MODE",
            1: "ACTIVE_MODE",
            2: "ACTIVE_CANCELLATION_MODE",
        }
        return pan_mode_dict.get(self._state["pan_mode"], "UNKNOWN_MODE")

    def pan(self):
        return self._state["pan"]

    def panning(self):
        return self._state["panning"]

    def set_pan(self, angle, speed=1.0, timeout=10.0, active_cancellation=False):
        if speed > HeadPanCommand.MAX_SPEED_RATIO:
            self._node.get_logger().warning(
                "Commanded speed above max; clamping to max"
            )
            speed = HeadPanCommand.MAX_SPEED_RATIO
        elif speed < HeadPanCommand.MIN_SPEED_RATIO:
            self._node.get_logger().warning(
                "Commanded speed below min; clamping to min"
            )
            speed = HeadPanCommand.MIN_SPEED_RATIO

        mode = HeadPanCommand.SET_ACTIVE_MODE
        if active_cancellation:
            # Active cancellation relies on transform behavior that has not yet been fully ported.
            self._node.get_logger().warning(
                "active_cancellation requested, but tf-based cancellation is not yet ported; using active mode"
            )

        msg = HeadPanCommand()
        msg.target = float(angle)
        msg.speed_ratio = float(speed)
        msg.pan_mode = mode

        self._pub_pan.publish(msg)

        if timeout == 0:
            return True

        wait_for_ros2(
            lambda: abs(self.pan() - angle) <= settings.HEAD_PAN_ANGLE_TOLERANCE,
            node=self._node,
            timeout=timeout,
            rate=100.0,
            timeout_msg=f"Failed to move head to pan command {angle}",
            body=lambda: self._pub_pan.publish(msg),
        )
        return True
