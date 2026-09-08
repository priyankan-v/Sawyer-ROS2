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
    def __init__(self, limb, rate_hz, mode, stopped_velocity_tolerance):
        node_name = f"sdk_{mode}_joint_trajectory_action_server_{limb}"
        super().__init__(node_name)
        self._limb = limb
        self._rate_hz = rate_hz
        self._mode = mode
        self._stopped_velocity_tolerance = float(max(0.0, stopped_velocity_tolerance))
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

    def _current_joint_state_map(self, joint_names):
        with self._lock:
            msg = self._latest_joint_state

        if msg is None:
            return None

        index = {name: i for i, name in enumerate(msg.name)}
        positions = {}
        velocities = {}
        for name in joint_names:
            idx = index.get(name)
            if idx is None:
                return None
            positions[name] = msg.position[idx] if idx < len(msg.position) else 0.0
            velocities[name] = msg.velocity[idx] if idx < len(msg.velocity) else 0.0

        return positions, velocities

    @staticmethod
    def _joint_tolerance_map(tolerances):
        mapped = {}
        for tol in tolerances:
            if not tol.name:
                continue
            mapped[tol.name] = {
                "position": float(tol.position),
                "velocity": float(tol.velocity),
                "acceleration": float(tol.acceleration),
            }
        return mapped

    def _validate_goal(self, goal):
        joint_names = list(goal.trajectory.joint_names)
        points = list(goal.trajectory.points)
        if not joint_names:
            return False, "trajectory.joint_names is empty"
        if not points:
            return False, "trajectory.points is empty"

        count = len(joint_names)
        prev_t = -1e-9
        for i, point in enumerate(points):
            t = duration_to_sec(point.time_from_start)
            if t < 0.0:
                return False, f"point {i} has negative time_from_start"
            if t < prev_t:
                return False, "trajectory times must be non-decreasing"
            prev_t = t

            if self._mode == "velocity":
                if point.velocities and len(point.velocities) != count:
                    return False, f"point {i} velocities length must be {count}"
            else:
                if len(point.positions) != count:
                    return False, f"point {i} positions length must be {count}"

        return True, ""

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

    def _check_path_tolerance(self, point, joint_names, pos_map, vel_map, tol_map):
        if not tol_map:
            return True, ""

        for i, joint in enumerate(joint_names):
            tol = tol_map.get(joint)
            if tol is None:
                continue

            p_tol = tol["position"]
            if p_tol > 0.0 and point.positions:
                p_err = abs(point.positions[i] - pos_map[joint])
                if p_err > p_tol:
                    return False, f"path tolerance violated for {joint}: {p_err:.6f} > {p_tol:.6f}"

            v_tol = tol["velocity"]
            if v_tol > 0.0 and point.velocities:
                v_err = abs(point.velocities[i] - vel_map[joint])
                if v_err > v_tol:
                    return False, f"velocity path tolerance violated for {joint}: {v_err:.6f} > {v_tol:.6f}"

        return True, ""

    def _goal_tolerances_satisfied(self, final_point, joint_names, pos_map, vel_map, tol_map):
        for i, joint in enumerate(joint_names):
            tol = tol_map.get(joint)

            p_tol = tol["position"] if tol else 0.0
            if p_tol > 0.0 and final_point.positions:
                p_err = abs(final_point.positions[i] - pos_map[joint])
                if p_err > p_tol:
                    return False, f"goal tolerance violated for {joint}: {p_err:.6f} > {p_tol:.6f}"

            v_tol = tol["velocity"] if tol and tol["velocity"] > 0.0 else self._stopped_velocity_tolerance
            if v_tol > 0.0:
                v_err = abs(vel_map[joint])
                if v_err > v_tol:
                    return False, f"stopped velocity tolerance violated for {joint}: {v_err:.6f} > {v_tol:.6f}"

        return True, ""

    def goal_callback(self, goal_request):
        valid, msg = self._validate_goal(goal_request)
        if not valid:
            self.get_logger().error(f"Rejecting goal: {msg}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        goal = goal_handle.request
        joint_names = list(goal.trajectory.joint_names)
        points = list(goal.trajectory.points)
        path_tol_map = self._joint_tolerance_map(goal.path_tolerance)
        goal_tol_map = self._joint_tolerance_map(goal.goal_tolerance)

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = joint_names

        result = FollowJointTrajectory.Result()
        start = time.monotonic()
        last_t = 0.0
        sleep_period = max(0.001, 1.0 / self._rate_hz)

        for point in points:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Goal canceled"
                return result

            target_t = duration_to_sec(point.time_from_start)
            if target_t < last_t:
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

            current = self._current_joint_state_map(joint_names)
            if current is None:
                actual_positions = [0.0] * len(joint_names)
                actual_velocities = [0.0] * len(joint_names)
            else:
                pos_map, vel_map = current
                ok, err = self._check_path_tolerance(point, joint_names, pos_map, vel_map, path_tol_map)
                if not ok:
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = err
                    return result
                actual_positions = [pos_map[name] for name in joint_names]
                actual_velocities = [vel_map[name] for name in joint_names]

            feedback.desired = point
            feedback.actual.positions = actual_positions
            feedback.actual.velocities = actual_velocities
            feedback.actual.time_from_start = point.time_from_start
            feedback.error.positions = [d - a for d, a in zip(point.positions, actual_positions)]
            if point.velocities:
                feedback.error.velocities = [d - a for d, a in zip(point.velocities, actual_velocities)]
            feedback.error.time_from_start = point.time_from_start
            goal_handle.publish_feedback(feedback)

            last_t = target_t

        final_point = points[-1]
        tolerance_window = duration_to_sec(goal.goal_time_tolerance)
        tolerance_deadline = duration_to_sec(final_point.time_from_start) + tolerance_window
        wait_start = time.monotonic()
        last_goal_err = "goal tolerance check timed out without joint state"

        while True:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                result.error_string = "Goal canceled"
                return result

            current = self._current_joint_state_map(joint_names)
            if current is not None:
                pos_map, vel_map = current
                ok, last_goal_err = self._goal_tolerances_satisfied(
                    final_point,
                    joint_names,
                    pos_map,
                    vel_map,
                    goal_tol_map,
                )
                if ok:
                    goal_handle.succeed()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = "Trajectory command sequence published and tolerances satisfied"
                    return result

            elapsed = time.monotonic() - start
            if elapsed > tolerance_deadline:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.GOAL_TOLERANCE_VIOLATED
                result.error_string = last_goal_err
                return result

            # If no tolerance window is requested, do a short best-effort settle check.
            if tolerance_window == 0.0 and (time.monotonic() - wait_start) > 0.5:
                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                result.error_string = "Trajectory command sequence published"
                return result

            await asyncio.sleep(sleep_period)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--limb", default="right")
    parser.add_argument("--rate", type=float, default=100.0)
    parser.add_argument("--mode", default="position", choices=["position", "velocity"])
    parser.add_argument("--stopped-velocity-tolerance", type=float, default=0.05)
    return parser.parse_known_args()[0]


def main():
    args = parse_args()
    rclpy.init()
    node = JointTrajectoryActionServerROS2(
        args.limb,
        args.rate,
        args.mode,
        args.stopped_velocity_tolerance,
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
