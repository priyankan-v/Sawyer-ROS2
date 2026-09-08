# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import threading
import time
import uuid

import rclpy
from rclpy.node import Node

from intera_core_msgs.msg import IODeviceConfiguration
from intera_dataflow.wait_for_ros2 import wait_for_ros2
from intera_io.io_interface_ros2 import IODeviceInterfaceROS2

from .robot_params_ros2 import RobotParamsROS2


class CuffROS2:
    """ROS 2 interface class for cuff state and callbacks."""

    def __init__(self, limb="right", node=None):
        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node(f"intera_cuff_{limb}_ros2")
            self._owns_node = True

        self._node = node
        params = RobotParamsROS2(self._node)
        limb_names = params.get_limb_names()
        if limb_names and limb not in limb_names:
            raise ValueError(f"Cannot detect cuff limb {limb}. Valid limbs are {limb_names}")

        self.limb = limb
        self.name = "cuff"
        self._device = None
        self._threads = {}
        self._stop_events = {}

        self._cuff_config_sub = self._node.create_subscription(
            IODeviceConfiguration,
            "/io/robot/cuff/config",
            self._config_callback,
            10,
        )

        wait_for_ros2(
            lambda: self._device is not None,
            node=self._node,
            timeout=5.0,
            timeout_msg=f"Failed to find cuff on limb '{limb}'",
            raise_on_error=False,
        )

        self._cuff_io = IODeviceInterfaceROS2(
            node=self._node,
            node_name="robot",
            dev_name=self.name,
            require_config=False,
            require_state=False,
        )

    def destroy(self):
        for callback_id in list(self._threads.keys()):
            self.deregister_callback(callback_id)
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def _config_callback(self, msg):
        if msg.device.name and str(msg.device.name) == self.name:
            self._device = msg.device

    def lower_button(self):
        return bool(self._cuff_io.get_signal_value(f"{self.limb}_button_lower"))

    def upper_button(self):
        return bool(self._cuff_io.get_signal_value(f"{self.limb}_button_upper"))

    def cuff_button(self):
        return bool(self._cuff_io.get_signal_value(f"{self.limb}_cuff"))

    def register_callback(self, callback_function, signal_name, poll_rate=10):
        callback_id = str(uuid.uuid4())
        stop_event = threading.Event()
        self._stop_events[callback_id] = stop_event

        def signal_spinner():
            old_state = self._cuff_io.get_signal_value(signal_name)
            period = 0.1 if poll_rate <= 0 else (1.0 / float(poll_rate))
            while self._node.context.ok() and not stop_event.is_set():
                new_state = self._cuff_io.get_signal_value(signal_name)
                if new_state != old_state:
                    callback_function(new_state)
                old_state = new_state
                time.sleep(period)

        t = threading.Thread(target=signal_spinner)
        t.daemon = True
        t.start()
        self._threads[callback_id] = t
        return callback_id

    def deregister_callback(self, callback_id):
        if callback_id not in self._threads:
            return False

        self._stop_events[callback_id].set()
        self._threads[callback_id].join(timeout=1.0)
        del self._threads[callback_id]
        del self._stop_events[callback_id]
        return True
