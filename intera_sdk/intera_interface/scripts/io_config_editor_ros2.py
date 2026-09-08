#!/usr/bin/env python3

import argparse
import json

import rclpy
from rclpy.node import Node

from intera_core_msgs.msg import IOComponentCommand, IONodeConfiguration
from intera_dataflow.wait_for_ros2 import wait_for_ros2


class EndEffectorConfigEditorROS2:
    def __init__(self, node):
        self._node = node
        self._config_msg = None
        self._pub = self._node.create_publisher(IOComponentCommand, "/io/end_effector/command", 10)
        self._node.create_subscription(IONodeConfiguration, "/io/end_effector/config", self._on_config, 10)

    def _on_config(self, msg):
        self._config_msg = msg

    def _wait_for_config(self):
        return wait_for_ros2(
            lambda: self._config_msg is not None and len(self._config_msg.devices) > 0,
            node=self._node,
            timeout=5.0,
            raise_on_error=False,
        )

    def _find_device(self, device_name=None):
        if not self._wait_for_config():
            raise RuntimeError("No end-effector configuration received on /io/end_effector/config")

        devices = list(self._config_msg.devices)
        if device_name:
            for dev in devices:
                if str(dev.name) == device_name:
                    return dev
            raise RuntimeError(f"Device '{device_name}' not found in end-effector config")

        return devices[0]

    def save_config(self, filename, device_name=None):
        device = self._find_device(device_name)
        raw = json.loads(device.config)
        with open(filename, "w", encoding="utf-8") as f:
            json.dump(raw, f, ensure_ascii=True, sort_keys=True, indent=2, separators=(",", ": "))
        self._node.get_logger().info(f"Saved end-effector config for '{device.name}' to {filename}")

    def load_config(self, filename, device_name=None):
        device = self._find_device(device_name)
        with open(filename, "r", encoding="utf-8") as f:
            config_data = json.load(f)

        msg = IOComponentCommand()
        msg.time = self._node.get_clock().now().to_msg()
        msg.op = "reconfigure"
        msg.args = json.dumps({"devices": {str(device.name): config_data}, "write_config": True})
        self._pub.publish(msg)
        self._node.get_logger().info(f"Sent reconfigure request for '{device.name}' from {filename}")


def main():
    parser = argparse.ArgumentParser(description="Save/load end-effector config (ROS2)")
    parser.add_argument("-s", "--save", metavar="PATH", help="save current EE config to file")
    parser.add_argument("-l", "--load", metavar="PATH", help="load EE config from file")
    parser.add_argument("-d", "--device", metavar="NAME", help="target end-effector device name")
    args = parser.parse_args()

    if not args.save and not args.load:
        parser.print_usage()
        print("No action defined")
        return 0

    rclpy.init()
    node = Node("ee_config_editor_ros2")
    try:
        editor = EndEffectorConfigEditorROS2(node)
        if args.save:
            editor.save_config(args.save, args.device)
        if args.load:
            editor.load_config(args.load, args.device)
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
