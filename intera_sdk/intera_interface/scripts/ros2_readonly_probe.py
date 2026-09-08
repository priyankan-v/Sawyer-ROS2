#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.node import Node

from intera_interface.joint_limits_ros2 import JointLimitsROS2
from intera_interface.robot_params_ros2 import RobotParamsROS2


class ReadonlyProbeNode(Node):
    def __init__(self, timeout_sec):
        super().__init__("intera_readonly_probe")
        self._params = RobotParamsROS2(self)
        self._joint_limits = JointLimitsROS2(self, timeout=timeout_sec)

    def report(self):
        robot_name = self._params.get_robot_name()
        assemblies = self._params.get_robot_assemblies()
        limbs = self._params.get_limb_names()
        cameras = self._params.get_camera_names()
        limits = self._joint_limits.joint_velocity_limits()

        self.get_logger().info(f"robot_name={robot_name}")
        self.get_logger().info(f"assemblies={assemblies}")
        self.get_logger().info(f"limbs={limbs}")
        self.get_logger().info(f"cameras={cameras}")
        self.get_logger().info(f"joint_velocity_limits_count={len(limits)}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    rclpy.init()
    node = ReadonlyProbeNode(timeout_sec=args.timeout)
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
