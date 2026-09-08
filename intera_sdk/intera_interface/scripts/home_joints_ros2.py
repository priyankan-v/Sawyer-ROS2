#!/usr/bin/env python3

import argparse
import threading

import rclpy
from rclpy.node import Node

import intera_interface
from intera_core_msgs.msg import HomingCommand, HomingState, RobotAssemblyState


class HomeJointsROS2:
    def __init__(self, node):
        self._node = node
        self._homing_lock = threading.Lock()
        self._enable_lock = threading.Lock()
        self._homing_state = {}
        self._enabled = False

        self._pub_home = self._node.create_publisher(HomingCommand, "/robot/set_homing_mode", 10)
        self._node.create_subscription(RobotAssemblyState, "/robot/state", self._enable_state_cb, 10)
        self._node.create_subscription(HomingState, "/robot/homing_states", self._homing_state_cb, 10)

    def _enable_state_cb(self, msg):
        with self._enable_lock:
            self._enabled = bool(msg.enabled)

    def _homing_state_cb(self, msg):
        with self._homing_lock:
            self._homing_state = dict(zip(msg.name, msg.state))

    def _robot_is_disabled(self):
        with self._enable_lock:
            return not self._enabled

    def _joints_are_homed(self):
        with self._homing_lock:
            if not self._homing_state:
                return False
            for _, value in self._homing_state.items():
                if value != HomingState.HOMED:
                    return False
            return True

    def home_robot(self, mode=HomingCommand.AUTO, timeout=60.0):
        start = self._node.get_clock().now().nanoseconds / 1e9

        def timeout_reached():
            now = self._node.get_clock().now().nanoseconds / 1e9
            return (now - start) > timeout

        while self._node.context.ok() and self._robot_is_disabled() and not timeout_reached():
            rclpy.spin_once(self._node, timeout_sec=0.1)

        homing_joints = []
        while self._node.context.ok() and not timeout_reached():
            rclpy.spin_once(self._node, timeout_sec=0.1)
            with self._homing_lock:
                if self._homing_state:
                    homing_joints = list(self._homing_state.keys())
                    break

        if not homing_joints:
            return False

        cmd = HomingCommand()
        cmd.name = homing_joints
        cmd.command = [mode] * len(homing_joints)

        while self._node.context.ok() and not timeout_reached():
            if self._joints_are_homed():
                return True
            self._pub_home.publish(cmd)
            rclpy.spin_once(self._node, timeout_sec=0.25)

        return False


def main():
    parser = argparse.ArgumentParser(description="Home robot joints (ROS2)")
    parser.add_argument("-t", "--timeout", type=lambda t: abs(float(t)), default=60.0)
    parser.add_argument("-m", "--mode", type=str.upper, default="AUTO", choices=["AUTO", "MANUAL"])
    enable_parser = parser.add_mutually_exclusive_group(required=False)
    enable_parser.add_argument("-e", "--enable", action="store_true", dest="enable", help="try to enable robot before homing")
    enable_parser.add_argument("-n", "--no-enable", action="store_false", dest="enable", help="avoid trying to enable robot")
    enable_parser.set_defaults(enable=True)
    args = parser.parse_args()

    rclpy.init()
    node = Node("home_joints_ros2")
    try:
        if args.enable:
            rs = intera_interface.RobotEnableROS2(node=node, versioned=False)
            rs.enable()

        cmd_mode = HomingCommand.MANUAL if args.mode == "MANUAL" else HomingCommand.AUTO
        node.get_logger().info(f"Homing joints in '{args.mode.capitalize()}' mode")
        homer = HomeJointsROS2(node)
        success = homer.home_robot(mode=cmd_mode, timeout=args.timeout)
        node.get_logger().info(f"{'Succeeded' if success else 'Failed'} in homing the robot's joints")
        return 0 if success else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
