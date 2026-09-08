# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import copy
import json

from intera_core_msgs.msg import IOComponentCommand, IODeviceConfiguration, IODeviceStatus
from intera_dataflow.wait_for_ros2 import wait_for_ros2

from .io_command_ros2 import SetCommandROS2


class IOInterfaceROS2:
    """Base ROS 2 class for IO interfaces."""

    def __init__(self, node, path_root, config_msg_type, status_msg_type, require_config=True, require_state=False):
        self._node = node
        self._path = path_root
        self.ports = {}
        self.signals = {}
        self.config = config_msg_type()
        self.state = status_msg_type()
        self._config_valid = False
        self._state_valid = False

        self._config_sub = self._node.create_subscription(
            config_msg_type,
            self._path + "/config",
            self.handle_config,
            10,
        )
        self._state_sub = self._node.create_subscription(
            status_msg_type,
            self._path + "/state",
            self.handle_state,
            10,
        )
        self._command_pub = self._node.create_publisher(IOComponentCommand, self._path + "/command", 10)

        if require_config:
            wait_for_ros2(
                lambda: self._config_valid,
                node=self._node,
                timeout=5.0,
                timeout_msg=f"Failed to get config at: {self._path}/config",
            )

        if require_state:
            is_init = wait_for_ros2(
                lambda: self._state_valid,
                node=self._node,
                timeout=5.0,
                raise_on_error=False,
            )
            if not is_init:
                self._node.get_logger().info(
                    f"Did not receive initial state at: {self._path}/state. Device may not be activated yet."
                )

    def is_config_valid(self):
        return self._config_valid

    def is_state_valid(self):
        return self._state_valid

    def is_valid(self):
        return self._config_valid and self._state_valid

    def handle_config(self, msg):
        self.config = msg
        self._config_valid = True

    def _load_state(self, current_state, incoming_state):
        for state in incoming_state:
            if state.name not in current_state:
                current_state[state.name] = {}
            formatting = json.loads(state.format)
            current_state[state.name]["type"] = formatting.get("type")
            current_state[state.name]["role"] = formatting.get("role")
            data = json.loads(state.data)
            current_state[state.name]["data"] = data[0] if len(data) > 0 else None

    def handle_state(self, msg):
        self.state = msg
        self._state_valid = True
        self._load_state(self.ports, self.state.ports)
        self._load_state(self.signals, self.state.signals)

    def publish_command(self, op, args):
        cmd_msg = IOComponentCommand()
        cmd_msg.time = self._node.get_clock().now().to_msg()
        cmd_msg.op = op
        cmd_msg.args = json.dumps(args)
        self._command_pub.publish(cmd_msg)
        return True


class IODeviceInterfaceROS2(IOInterfaceROS2):
    """ROS 2 IO Device interface to config, status, and command topics."""

    def __init__(self, node, node_name, dev_name, require_config=True, require_state=False):
        super().__init__(
            node,
            "/io/" + node_name + "/" + dev_name,
            IODeviceConfiguration,
            IODeviceStatus,
            require_config=require_config,
            require_state=require_state,
        )

    def list_signal_names(self):
        return copy.deepcopy(list(self.signals.keys()))

    def get_signal_type(self, signal_name):
        if signal_name in self.signals:
            return copy.deepcopy(self.signals[signal_name].get("type"))
        return None

    def get_signal_value(self, signal_name):
        if signal_name in self.signals:
            return copy.deepcopy(self.signals[signal_name].get("data"))
        return None

    def set_signal_value(self, signal_name, signal_value, signal_type=None):
        s_type = signal_type
        if s_type is None:
            s_type = self.get_signal_type(signal_name)
        if s_type is None:
            # Fallback type inference keeps control flows usable even before state sync.
            if isinstance(signal_value, bool):
                s_type = "bool"
            elif isinstance(signal_value, int):
                s_type = "int32"
            elif isinstance(signal_value, float):
                s_type = "float"
            else:
                s_type = "string"

        set_command = SetCommandROS2().set_signal(signal_name, s_type, signal_value)
        msg = set_command.as_msg(node=self._node)
        self._command_pub.publish(msg)
        return True
