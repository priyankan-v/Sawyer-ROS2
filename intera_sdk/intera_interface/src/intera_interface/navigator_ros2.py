# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import threading
import time
import uuid

import rclpy
from rclpy.node import Node

from intera_io.io_interface_ros2 import IODeviceInterfaceROS2


class NavigatorROS2:
    """ROS 2 interface class for a Navigator on the Intera robot."""

    def __init__(self, node=None):
        self._owns_node = False
        if node is None:
            if not rclpy.ok():
                rclpy.init()
            node = Node("intera_navigator_ros2")
            self._owns_node = True

        self._node = node
        self._navigator_io = IODeviceInterfaceROS2(
            node=self._node,
            node_name="robot",
            dev_name="navigator",
            require_config=False,
            require_state=False,
        )
        self._button_lookup = {
            0: "OFF",
            1: "CLICK",
            2: "LONG_PRESS",
            3: "DOUBLE_CLICK",
        }

        self._threads = {}
        self._stop_events = {}

    def destroy(self):
        for callback_id in list(self._threads.keys()):
            self.deregister_callback(callback_id)
        if self._owns_node and self._node is not None:
            self._node.destroy_node()
            self._node = None

    def list_all_items(self):
        return self._navigator_io.list_signal_names()

    def get_wheel_state(self, wheel_name):
        return self._get_item_state(wheel_name)

    def get_button_state(self, button_name):
        return self._get_item_state(button_name)

    def button_string_lookup(self, button_value):
        return self._button_lookup.get(button_value, "INVALID_VALUE")

    def register_callback(self, callback_function, signal_name, poll_rate=10):
        if signal_name not in self.list_all_items():
            self._node.get_logger().warning(
                f"Signal '{signal_name}' not currently present in navigator state"
            )

        callback_id = str(uuid.uuid4())
        stop_event = threading.Event()
        self._stop_events[callback_id] = stop_event

        def signal_spinner():
            old_state = self.get_button_state(signal_name)
            period = 0.1 if poll_rate <= 0 else (1.0 / float(poll_rate))
            while self._node.context.ok() and not stop_event.is_set():
                new_state = self.get_button_state(signal_name)
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

    def _get_item_state(self, item_name):
        return self._navigator_io.get_signal_value(item_name)
