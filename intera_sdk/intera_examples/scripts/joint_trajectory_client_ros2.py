#!/usr/bin/env python3

import argparse

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class JointTrajectoryClientROS2(Node):
    def __init__(self, limb, points_scale):
        super().__init__(f"sdk_joint_trajectory_client_{limb}")
        self._limb = limb
        self._points_scale = points_scale
        self._joint_names = [f"{limb}_j{i}" for i in range(7)]

        topic = f"/robot/limb/{limb}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, topic)

    def _point(self, positions, t_sec):
        p = JointTrajectoryPoint()
        p.positions = positions
        p.time_from_start.sec = int(t_sec)
        p.time_from_start.nanosec = int((t_sec - int(t_sec)) * 1e9)
        return p

    def _build_goal(self):
        goal = FollowJointTrajectory.Goal()
        goal.goal_time_tolerance.sec = 0
        goal.goal_time_tolerance.nanosec = int(0.1 * 1e9)
        goal.trajectory.joint_names = list(self._joint_names)

        base = [0.0] * len(self._joint_names)
        s = self._points_scale
        t = 1.0
        goal.trajectory.points.append(self._point(base, 0.0))
        goal.trajectory.points.append(self._point(base, t))
        t += 3.0
        goal.trajectory.points.append(self._point([x + 0.6 * s for x in base], t))
        t += 3.0
        goal.trajectory.points.append(self._point([x + 0.3 * s for x in base], t))
        t += 3.0
        goal.trajectory.points.append(self._point([x + 0.6 * s for x in base], t))
        t += 3.0
        goal.trajectory.points.append(self._point([x + 0.3 * s for x in base], t))
        t += 3.0
        goal.trajectory.points.append(self._point([x + 0.6 * s for x in base], t))
        t += 3.0
        goal.trajectory.points.append(self._point(base, t))
        return goal, t

    def run(self):
        self.get_logger().info("Waiting for trajectory action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Timed out waiting for action server")
            return 1

        goal, timeout = self._build_goal()
        self.get_logger().info("Sending trajectory goal")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return 1

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout + 5.0)
        result = result_future.result()
        if result is None:
            self.get_logger().error("Timed out waiting for trajectory result")
            return 1

        self.get_logger().info(f"Trajectory finished with code {result.result.error_code}")
        return 0


def parse_args():
    parser = argparse.ArgumentParser(description="ROS2 Joint Trajectory Action Client")
    parser.add_argument("--limb", default="right", choices=["right", "left"])
    parser.add_argument("--points-scale", type=float, default=0.1)
    return parser.parse_known_args()[0]


def main():
    args = parse_args()
    rclpy.init()
    node = JointTrajectoryClientROS2(args.limb, args.points_scale)
    try:
        raise SystemExit(node.run())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
