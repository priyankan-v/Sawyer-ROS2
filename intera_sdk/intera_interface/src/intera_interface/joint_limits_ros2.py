# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from copy import deepcopy

from intera_core_msgs.msg import JointLimits as JointLimitsMsg
from intera_dataflow.wait_for_ros2 import wait_for_ros2


class JointLimitsROS2:
    """Read-only joint limits interface for ROS 2."""

    def __init__(self, node, joint_limit_topic="/robot/joint_limits", timeout=5.0):
        self._node = node
        self._joint_position_lower = {}
        self._joint_position_upper = {}
        self._joint_velocity_limit = {}
        self._joint_accel_limit = {}
        self._joint_effort_limit = {}
        self._joint_names = []

        self._joint_limit_sub = self._node.create_subscription(
            JointLimitsMsg,
            joint_limit_topic,
            self._on_joint_limits,
            1,
        )

        err_msg = f"init failed to get current joint_limits from {joint_limit_topic}"
        wait_for_ros2(
            lambda: len(self._joint_names) > 0,
            node=self._node,
            timeout=timeout,
            timeout_msg=err_msg,
        )

    def _on_joint_limits(self, msg):
        self._joint_names = list(msg.joint_names)
        self._joint_position_lower.clear()
        self._joint_position_upper.clear()
        self._joint_velocity_limit.clear()
        self._joint_accel_limit.clear()
        self._joint_effort_limit.clear()

        for idx, name in enumerate(msg.joint_names):
            self._joint_position_lower[name] = msg.position_lower[idx]
            self._joint_position_upper[name] = msg.position_upper[idx]
            self._joint_velocity_limit[name] = msg.velocity[idx]
            self._joint_accel_limit[name] = msg.accel[idx]
            self._joint_effort_limit[name] = msg.effort[idx]

    def joint_position_lower_limits(self):
        return deepcopy(self._joint_position_lower)

    def joint_position_upper_limits(self):
        return deepcopy(self._joint_position_upper)

    def joint_velocity_limits(self):
        return deepcopy(self._joint_velocity_limit)

    def joint_acceleration_limits(self):
        return deepcopy(self._joint_accel_limit)

    def joint_effort_limits(self):
        return deepcopy(self._joint_effort_limit)

    def joint_lower_limit(self, joint):
        return self._joint_position_lower[joint]

    def joint_upper_limit(self, joint):
        return self._joint_position_upper[joint]

    def joint_velocity_limit(self, joint):
        return self._joint_velocity_limit[joint]

    def joint_acceleration_limit(self, joint):
        return self._joint_accel_limit[joint]

    def joint_effort_limit(self, joint):
        return self._joint_effort_limit[joint]

    def get_joint_lower_limits(self, joint_names):
        return [self._joint_position_lower[name] for name in joint_names]

    def get_joint_upper_limits(self, joint_names):
        return [self._joint_position_upper[name] for name in joint_names]

    def get_joint_velocity_limits(self, joint_names):
        return [self._joint_velocity_limit[name] for name in joint_names]

    def get_joint_acceleration_limits(self, joint_names):
        return [self._joint_accel_limit[name] for name in joint_names]

    def get_joint_effort_limits(self, joint_names):
        return [self._joint_effort_limit[name] for name in joint_names]
