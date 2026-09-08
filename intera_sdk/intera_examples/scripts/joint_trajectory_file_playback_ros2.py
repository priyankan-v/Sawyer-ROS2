#!/usr/bin/env python3

import argparse
import csv
import math
import os
import time
from bisect import bisect

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

import intera_interface
from intera_interface.robot_params_ros2 import RobotParamsROS2


class TrajectoryFilePlaybackROS2(Node):
    def __init__(self, limb, filename, auto_enable=True):
        super().__init__(f"sdk_joint_trajectory_file_playback_{limb}")
        self._limb = limb
        self._filename = filename
        self._auto_enable = auto_enable
        self._gripper_rate_hz = 20.0
        self._limb_iface = None
        self._gripper = None

        topic = f"/robot/limb/{limb}/follow_joint_trajectory"
        self._client = ActionClient(self, FollowJointTrajectory, topic)

    def _duration_from_seconds(self, t_sec):
        p = JointTrajectoryPoint()
        p.time_from_start.sec = int(math.floor(t_sec))
        p.time_from_start.nanosec = int((t_sec - math.floor(t_sec)) * 1e9)
        return p.time_from_start

    def _find_start_offset(self, first_command):
        current = self._limb_iface.joint_angles()
        max_offset = 0.0
        default_vel = 0.25
        for joint, cmd in first_command.items():
            cur = current.get(joint)
            if cur is None:
                continue
            max_offset = max(max_offset, abs(cmd - cur) / default_vel)
        return max_offset

    def _load_csv(self):
        with open(self._filename, "r", encoding="utf-8") as f:
            reader = csv.reader(f)
            rows = [row for row in reader if row]

        if len(rows) < 2:
            raise ValueError("Trajectory file must include header plus at least one data row")

        header = [h.strip() for h in rows[0]]
        if not header or header[0].lower() not in ("time", "timestamp"):
            raise ValueError("First column must be time")

        data_rows = []
        for raw in rows[1:]:
            if len(raw) < len(header):
                raw = raw + [""] * (len(header) - len(raw))
            t = float(raw[0])
            values = {}
            for idx, name in enumerate(header[1:], start=1):
                val = raw[idx].strip() if idx < len(raw) else ""
                if val == "":
                    continue
                values[name] = float(val)
            data_rows.append((t, values))

        if not data_rows:
            raise ValueError("No trajectory points found")

        return header, data_rows

    def _build_goal_and_gripper_track(self):
        rp = RobotParamsROS2(self)
        limb_names = rp.get_limb_names() or ["right"]
        if self._limb not in limb_names:
            self.get_logger().warning(f"Limb '{self._limb}' not in detected limbs: {limb_names}")

        self._limb_iface = intera_interface.LimbROS2(
            limb=self._limb,
            node=self,
            require_joint_states=True,
            require_endpoint_state=False,
            require_tip_states=False,
        )

        rs = intera_interface.RobotEnableROS2(node=self, versioned=False)
        if self._auto_enable:
            state = rs.state()
            if state is not None and not state.enabled:
                self.get_logger().info("Enabling robot...")
                rs.enable()

        gripper_name = f"{self._limb}_gripper"
        try:
            self._gripper = intera_interface.GripperROS2(gripper_name, node=self)
            if self._gripper.has_error():
                self._gripper.reboot()
            if not self._gripper.is_calibrated():
                self._gripper.calibrate()
        except Exception as exc:
            self._gripper = None
            self.get_logger().warning(f"No usable electric gripper for playback: {exc}")

        _, data_rows = self._load_csv()
        joint_names = self._limb_iface.joint_names()
        first = data_rows[0][1]
        first_joint_cmd = {j: first[j] for j in joint_names if j in first}
        start_offset = self._find_start_offset(first_joint_cmd)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joint_names)

        current_point = JointTrajectoryPoint()
        current_point.positions = [self._limb_iface.joint_angle(j) for j in joint_names]
        current_point.time_from_start = self._duration_from_seconds(0.0)
        goal.trajectory.points.append(current_point)

        gripper_track = []
        for t_rel, values in data_rows:
            positions = []
            for j in joint_names:
                if j in values:
                    positions.append(values[j])
                else:
                    prev = goal.trajectory.points[-1].positions[len(positions)]
                    positions.append(prev)

            p = JointTrajectoryPoint()
            p.positions = positions
            p.time_from_start = self._duration_from_seconds(t_rel + start_offset)
            goal.trajectory.points.append(p)

            if self._gripper and gripper_name in values:
                gripper_track.append((t_rel + start_offset, values[gripper_name]))

        return goal, gripper_track

    def _execute_gripper_track(self, gripper_track, result_future):
        if not self._gripper or not gripper_track:
            return

        times = [t for t, _ in gripper_track]
        start = time.monotonic()
        period = 1.0 / self._gripper_rate_hz
        while self.context.ok() and not result_future.done():
            elapsed = time.monotonic() - start
            idx = bisect(times, elapsed) - 1
            if idx >= 0:
                self._gripper.set_position(gripper_track[idx][1])
            rclpy.spin_once(self, timeout_sec=period)

    def run_once(self):
        self.get_logger().info("Waiting for trajectory action server...")
        if not self._client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("Timed out waiting for action server")
            return False

        goal, gripper_track = self._build_goal_and_gripper_track()
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("Trajectory goal was rejected")
            return False

        result_future = goal_handle.get_result_async()
        self._execute_gripper_track(gripper_track, result_future)

        # Keep spinning until result is available.
        while self.context.ok() and not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = result_future.result()
        if result is None:
            self.get_logger().error("No result from trajectory action")
            return False

        ok = result.result.error_code == 0
        if not ok:
            self.get_logger().warning(
                f"Trajectory action ended with error code {result.result.error_code}: {result.result.error_string}"
            )
        return ok


def parse_args():
    parser = argparse.ArgumentParser(description="SDK Joint Trajectory Example: file playback (ROS2)")
    parser.add_argument("-l", "--limb", default="right", choices=["right", "left"])
    parser.add_argument("-f", "--file", metavar="PATH", required=True, help="path to input file")
    parser.add_argument("-n", "--number_loops", type=int, default=1, help="number of playback loops. 0=infinite.")
    parser.add_argument("--no-auto-enable", action="store_true", help="do not auto-enable robot")
    return parser.parse_args()


def main():
    args = parse_args()
    filename = os.path.expanduser(args.file)
    if not os.path.isfile(filename):
        print(f"Input file does not exist: {filename}")
        return 1

    rclpy.init()
    node = TrajectoryFilePlaybackROS2(args.limb, filename, auto_enable=(not args.no_auto_enable))
    try:
        loop_count = 1
        max_loops = float("inf") if args.number_loops == 0 else max(1, args.number_loops)

        ok = True
        while ok and loop_count <= max_loops and node.context.ok():
            node.get_logger().info(
                f"Playback loop {loop_count} of {'forever' if math.isinf(max_loops) else int(max_loops)}"
            )
            try:
                ok = node.run_once()
            except Exception as exc:
                node.get_logger().error(f"Playback failed: {exc}")
                ok = False
            loop_count += 1

        if ok:
            node.get_logger().info("Exiting - File Playback Complete")
            return 0

        node.get_logger().warning("Exiting - File Playback Incomplete")
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
