# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import errno

import rclpy
from rclpy.node import Node

from intera_core_msgs.msg import IONodeConfiguration
from intera_dataflow.wait_for_ros2 import wait_for_ros2
from intera_io.io_interface_ros2 import IODeviceInterfaceROS2


class GripperROS2:
    """ROS 2 interface class for a gripper on the Intera robot."""

    MAX_POSITION = 0.041667
    MIN_POSITION = 0.0
    MAX_VELOCITY = 3.0
    MIN_VELOCITY = 0.15

    def __init__(self, ee_name="right_gripper", calibrate=True, node=None):
        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node("intera_gripper_ros2")
            self._owns_node = True

        self._node = node
        self._devices_msg = None
        self.name = ee_name
        if ee_name in ("right", "left"):
            self._node.get_logger().warning(
                "Specifying gripper by side is deprecated; use full name like right_gripper"
            )
            self.name = f"{ee_name}_gripper"

        self._ee_config_sub = self._node.create_subscription(
            IONodeConfiguration,
            "/io/end_effector/config",
            self._config_callback,
            10,
        )

        got_device = wait_for_ros2(
            lambda: self._devices_msg is not None,
            node=self._node,
            timeout=5.0,
            timeout_msg="Failed to get gripper. No gripper attached on the robot.",
            raise_on_error=False,
        )
        if not got_device:
            raise OSError(errno.ENOENT, "No matching gripper device configuration")

        self.gripper_io = IODeviceInterfaceROS2(
            node=self._node,
            node_name="end_effector",
            dev_name=self.name,
            require_config=False,
            require_state=False,
        )

        if self.has_error():
            self.reboot()
            calibrate = True
        if calibrate and not self.is_calibrated():
            self.calibrate()

    def destroy(self):
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def _config_callback(self, msg):
        for device in msg.devices:
            if str(device.name) == self.name:
                self._devices_msg = msg
                return

    def reboot(self):
        self.gripper_io.set_signal_value("reboot", True, signal_type="bool")

    def stop(self):
        self.gripper_io.set_signal_value("go", False, signal_type="bool")

    def start(self):
        self.gripper_io.set_signal_value("go", True, signal_type="bool")

    def open(self, position=MAX_POSITION):
        self.gripper_io.set_signal_value("position_m", float(position), signal_type="float")

    def close(self, position=MIN_POSITION):
        self.gripper_io.set_signal_value("position_m", float(position), signal_type="float")

    def has_error(self):
        return bool(self.gripper_io.get_signal_value("has_error"))

    def is_ready(self):
        return self.is_calibrated() and (not self.has_error()) and (not self.is_moving())

    def is_moving(self):
        return bool(self.gripper_io.get_signal_value("is_moving"))

    def is_gripping(self):
        return bool(self.gripper_io.get_signal_value("is_gripping"))

    def is_calibrated(self):
        return bool(self.gripper_io.get_signal_value("is_calibrated"))

    def calibrate(self):
        self.gripper_io.set_signal_value("calibrate", True, signal_type="bool")
        success = wait_for_ros2(
            lambda: self.is_calibrated(),
            node=self._node,
            timeout=5.0,
            raise_on_error=False,
        )
        if not success:
            self._node.get_logger().error(f"({self.name}) calibration failed")
        return success

    def get_position(self):
        return self.gripper_io.get_signal_value("position_response_m")

    def set_position(self, position):
        self.gripper_io.set_signal_value("position_m", float(position), signal_type="float")

    def set_velocity(self, speed):
        self._node.get_logger().warning(
            "set_velocity is deprecated. Use set_cmd_velocity instead"
        )
        self.set_cmd_velocity(speed)

    def set_cmd_velocity(self, speed):
        self.gripper_io.set_signal_value("speed_mps", float(speed), signal_type="float")

    def get_cmd_velocity(self):
        return self.gripper_io.get_signal_value("speed_mps")

    def get_force(self):
        return self.gripper_io.get_signal_value("force_response_n")

    def set_object_weight(self, object_weight):
        self.gripper_io.set_signal_value(
            self.name + "_tip_object_kg",
            float(object_weight),
            signal_type="float",
        )

    def get_object_weight(self):
        return self.gripper_io.get_signal_value(self.name + "_tip_object_kg")

    def set_dead_zone(self, dead_zone):
        self.gripper_io.set_signal_value("dead_zone_m", float(dead_zone), signal_type="float")

    def get_dead_zone(self):
        return self.gripper_io.get_signal_value("dead_zone_m")

    def set_holding_force(self, holding_force):
        del holding_force
        self._node.get_logger().error(
            "Removed variable holding force to improve gripper performance"
        )
        return False
