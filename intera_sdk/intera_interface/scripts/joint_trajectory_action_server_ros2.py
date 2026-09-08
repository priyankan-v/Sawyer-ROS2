#!/usr/bin/env python3

import argparse
import asyncio
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from intera_core_msgs.msg import JointCommand
from sensor_msgs.msg import JointState


def duration_to_sec(duration_msg):
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


class JointTrajectoryActionServerROS2(Node):
    def __init__(self, limb, rate_hz, mode):
        node_name = f"sdk_{mode}_joint_trajectory_action_server_{limb}"
        super().__init__(node_name)
        self._limb = limb
        self._rate_hz = rate_hz
        self._mode = mode
        self._lock = threading.Lock()
        self._latest_joint_state = None

        if self._mode not in ("position", "velocity"):
            raise ValueError("mode must be 'position' or 'velocity'")

        ns = f"/robot/limb/{limb}"
        self._joint_cmd_pub = self.create_publisher(
            JointCommand,
            f"{ns}/joint_command",
            10,
        )
        self.create_subscription(JointState, "/robot/joint_states", self._on_joint_state, 20)

        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f"{ns}/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        self.get_logger().info(f"ROS2 JointTrajectoryActionServer ready on {ns}/follow_joint_trajectory")

    def _on_joint_state(self, msg):
        with self._lock:
            self._latest_joint_state = msg

    def goal_callback(self, goal_request):
        if not goal_request.trajectory.joint_names:
            self.get_logger().error("Rejecting goal: trajectory.joint_names is empty")
            return GoalResponse.REJECT
        if not goal_request.trajectory.points:
            self.get_logger().error("Rejecting goal: trajectory.points is empty")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def _current_joint_positions(self, joint_names):
        with self._lock:
            msg = self._latest_joint_state

        if msg is None:
            return None

        index = {name: i for i, name in enumerate(msg.name)}
        positions = []
        for name in joint_names:
            if name not in index:
                return None
            positions.append(msg.position[index[name]])
        return positions

    def _publish_joint_command(self, joint_names, positions, velocities=None):
        cmd = JointCommand()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.names = list(joint_names)

        if self._mode == "velocity":
            cmd.mode = JointCommand.VELOCITY_MODE
            cmd.velocity = list(velocities if velocities is not None else [0.0] * len(joint_names))
        else:
            cmd.mode = JointCommand.POSITION_MODE
            cmd.position = list(positions)

        self._joint_cmd_pub.publish(cmd)

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        joint_names = list(goal.trajectory.joint_names)
        points = list(goal.trajectory.points)

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names

        result = FollowJointTrajectory.Result()

        start = time.monotonic()
        last_time = 0.0
        sleep_period = max(0.001, 1.0 / self._rate_hz)

        for point in points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Goal canceled"
                return result

            target_t = duration_to_sec(point.time_from_start)
            if target_t < last_time:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Trajectory times must be non-decreasing"
                return result

            while time.monotonic() - start < target_t:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                    result.error_string = "Goal canceled"
                    return result
                await asyncio.sleep(sleep_period)

            if self._mode == "velocity":
                velocities = point.velocities if point.velocities else [0.0] * len(joint_names)
                self._publish_joint_command(joint_names, positions=[], velocities=velocities)
            else:
                self._publish_joint_command(joint_names, point.positions)

            actual_positions = self._current_joint_positions(joint_names)
            if actual_positions is None:
                actual_positions = [0.0] * len(joint_names)

            feedback.desired = point
            feedback.actual.positions = actual_positions
            feedback.actual.time_from_start = point.time_from_start
            feedback.error.positions = [d - a for d, a in zip(point.positions, actual_positions)]
            feedback.error.time_from_start = point.time_from_start
            goal_handle.publish_feedback(feedback)

            last_time = target_t

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory command sequence published"
        return result


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--limb", default="right")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--mode", default="position", choices=["position", "velocity"])
    return parser.parse_known_args()[0]


def main():
    args = parse_args()
    rclpy.init()
    node = JointTrajectoryActionServerROS2(args.limb, args.rate, args.mode)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
