# Copyright (c) 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

import json

from intera_core_msgs.msg import IOComponentCommand


class IOCommandROS2:
    """Container for a generic ROS 2 IO command."""

    def __init__(self, op, args=None, stamp=None):
        self.stamp = stamp
        self.op = op
        self.args = args if args else {}

    def as_msg(self, *, node, stamp=None):
        msg = IOComponentCommand()
        msg.time = stamp if stamp is not None else (self.stamp if self.stamp is not None else node.get_clock().now().to_msg())
        msg.op = self.op
        msg.args = json.dumps(self.args)
        return msg


class SetCommandROS2(IOCommandROS2):
    """Container for a ROS 2 port or signal set command."""

    def __init__(self, args=None):
        super().__init__("set", args)

    def _set(self, components, component_name, data_type, dimensions, *component_value):
        self.args.setdefault(components, {})
        self.args[components][component_name] = {
            "format": {"type": data_type},
            "data": [val for val in component_value],
        }
        if dimensions > 1:
            self.args[components][component_name]["format"]["dimensions"] = [dimensions]

    def set_signal(self, signal_name, data_type, *signal_value):
        dimensions = len(signal_value)
        self._set("signals", signal_name, data_type, dimensions, *signal_value)
        return self

    def set_port(self, port_name, data_type, *port_value):
        dimensions = len(port_value)
        self._set("ports", port_name, data_type, dimensions, *port_value)
        return self
