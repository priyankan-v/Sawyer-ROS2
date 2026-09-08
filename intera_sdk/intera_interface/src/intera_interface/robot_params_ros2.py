# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

from rclpy.parameter import Parameter


class RobotParamsROS2:
    """Read-only access to robot configuration parameters in ROS 2.

    ROS 1 used a global parameter server with keys like
    /robot_config/assembly_names. In ROS 2, parameters are node-scoped.
    This helper accepts ROS 1 style parameter keys and maps them to the
    equivalent ROS 2 dotted form when needed.
    """

    def __init__(self, node):
        self._node = node

    @staticmethod
    def _to_ros2_param_name(name):
        stripped = name[1:] if name.startswith("/") else name
        return stripped.replace("/", ".")

    def _get_param(self, ros1_name, default):
        ros2_name = self._to_ros2_param_name(ros1_name)
        if not self._node.has_parameter(ros2_name):
            self._node.get_logger().warning(
                f"Parameter '{ros2_name}' not declared. Returning default."
            )
            return default

        parameter = self._node.get_parameter(ros2_name)
        if parameter.type_ == Parameter.Type.NOT_SET:
            self._node.get_logger().warning(
                f"Parameter '{ros2_name}' is not set. Returning default."
            )
            return default

        return parameter.value

    def get_camera_names(self):
        return list(self.get_camera_details().keys())

    def get_camera_details(self):
        return self._get_param("/robot_config/camera_config", {})

    def get_robot_assemblies(self):
        return self._get_param("/robot_config/assembly_names", [])

    def get_limb_names(self):
        non_limb_assemblies = {"torso", "head"}
        return list(set(self.get_robot_assemblies()).difference(non_limb_assemblies))

    def get_joint_names(self, limb_name):
        return self._get_param(f"/robot_config/{limb_name}_config/joint_names", [])

    def get_robot_name(self):
        return self._get_param("/manifest/robot_class", None)
