#!/usr/bin/env python3

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import intera_interface
from intera_core_msgs.action import CalibrationCommand


class CalibrateArmROS2(Node):
    def __init__(self, limb="right"):
        super().__init__(f"sdk_calibrate_arm_{limb}_ros2")
        self._limb = limb
        self._client = ActionClient(self, CalibrationCommand, "/calibration_command")

    def _feedback(self, feedback_msg):
        fb = feedback_msg.feedback
        ratio = 0.0
        if fb.number_of_poses > 0:
            ratio = float(fb.current_pose_number) / float(fb.number_of_poses)
        bar = ("#" * int(ratio * 40)).ljust(40)
        pct = f"{ratio * 100.0:.2f}".rjust(5, " ")
        state = str(fb.current_state).ljust(10)
        print(f"[{bar}] {pct}% complete - {state}", end="\r")

    def _send_calibration(self, command):
        goal = CalibrationCommand.Goal()
        goal.command = int(command)

        self.get_logger().info(f"Sending Calibration Request {command}.")
        send_future = self._client.send_goal_async(goal, feedback_callback=self._feedback)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError("Calibration goal was rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None:
            raise RuntimeError("Calibration action did not return a result")
        return result.result

    def stop_calibration(self):
        return self._send_calibration(CalibrationCommand.Goal.CALIBRATION_STOP)

    def start_calibration(self):
        rs = intera_interface.RobotEnableROS2(node=self, versioned=False)
        rs.enable()

        limb = intera_interface.LimbROS2(
            limb=self._limb,
            node=self,
            require_joint_states=True,
            require_endpoint_state=False,
            require_tip_states=False,
        )
        self.get_logger().info(f"Moving {self._limb} arm to neutral pose...")
        limb.move_to_neutral(timeout=25.0, speed=0.1)
        cal_result = self._send_calibration(CalibrationCommand.Goal.CALIBRATION_START)
        limb.set_joint_position_speed(speed=0.3)
        return cal_result


def is_gripper_removed(node):
    try:
        intera_interface.GripperROS2("right_gripper", node=node)
        node.get_logger().error("Calibration client: remove gripper attachments before calibration")
        return False
    except Exception:
        return True


def main():
    parser = argparse.ArgumentParser(description="Calibrate arm action client (ROS2)")
    parser.add_argument("-l", "--limb", choices=["left", "right"], default="right")
    args = parser.parse_args()

    rclpy.init()
    app = CalibrateArmROS2(args.limb)
    try:
        app.get_logger().info("Preparing to calibrate...")
        app.get_logger().info("IMPORTANT: remove grippers and other attachments before calibration")

        if not is_gripper_removed(app):
            return 1

        app.get_logger().info(f"Running calibrate on {args.limb} arm")
        error = None
        goal_state = "unreported error"
        try:
            if not app._client.wait_for_server(timeout_sec=10.0):
                app.get_logger().error("Timed out waiting for calibration action server")
                return 1
            goal_state = app.start_calibration()
        except KeyboardInterrupt as exc:
            error = exc
            goal_state = app.stop_calibration()

        result_text = str(goal_state).lower()
        if error is None and "success" in result_text:
            app.get_logger().info("Calibrate arm finished successfully. Reboot robot to use calibration data.")
            return 0

        app.get_logger().error(f"Calibrate arm failed with {goal_state}")
        app.get_logger().error("Please re-run this calibration request.")
        return 1
    finally:
        app.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
