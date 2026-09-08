# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import rclpy
from rclpy.node import Node

from intera_io.io_interface_ros2 import IODeviceInterfaceROS2


class LightsROS2:
    """ROS 2 interface class for robot lights."""

    def __init__(self, node=None):
        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node("intera_lights_ros2")
            self._owns_node = True

        self._node = node
        self._lights_io = IODeviceInterfaceROS2(
            node=self._node,
            node_name="robot",
            dev_name="robot",
            require_config=False,
            require_state=False,
        )

    def destroy(self):
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def list_all_lights(self):
        return [name for name in self._lights_io.list_signal_names() if "light" in name]

    def set_light_state(self, name, on=True):
        return self._lights_io.set_signal_value(name, bool(on), signal_type="bool")

    def get_light_state(self, name):
        return self._lights_io.get_signal_value(name)
