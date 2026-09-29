#!/usr/bin/env python3

import argparse
import math
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand


# =========================================================
# Sawyer right-arm joints
# =========================================================

RIGHT_JOINTS = [
    "right_j0",
    "right_j1",
    "right_j2",
    "right_j3",
    "right_j4",
    "right_j5",
    "right_j6",
]


# =========================================================
# Sawyer joint limits
#
# lower, upper, maximum velocity [rad, rad, rad/s]
# =========================================================

JOINT_LIMITS = {

    "right_j0": {
        "lower": -3.0503,
        "upper":  3.0503,
        "velocity": 1.740
    },

    "right_j1": {
        "lower": -3.8183,
        "upper":  2.2824,
        "velocity": 1.328
    },

    "right_j2": {
        "lower": -3.0514,
        "upper":  3.0514,
        "velocity": 1.957
    },

    "right_j3": {
        "lower": -3.0514,
        "upper":  3.0514,
        "velocity": 1.957
    },

    "right_j4": {
        "lower": -2.9842,
        "upper":  2.9842,
        "velocity": 3.485
    },

    "right_j5": {
        "lower": -2.9842,
        "upper":  2.9842,
        "velocity": 3.485
    },

    "right_j6": {
        "lower": -4.7104,
        "upper":  4.7104,
        "velocity": 4.545
    },
}


# =========================================================
# Controller settings
# =========================================================

CONTROL_RATE_HZ = 100.0

# Final position tolerance
POSITION_TOLERANCE = 0.01

# After reference trajectory finishes, allow the robot
# this much additional time to settle.
SETTLING_TIMEOUT = 3.0

# Abort if JointState feedback disappears.
JOINT_STATE_TIMEOUT = 0.25


class SawyerSingleJointTrajectory(Node):

    def __init__(
        self,
        test_joint,
        move_mode,
        move_value,
        trajectory_time
    ):

        super().__init__(
            "sawyer_single_joint_trajectory"
        )

        # =================================================
        # Requested motion
        # =================================================

        self.test_joint = test_joint

        # "relative" or "absolute"
        self.move_mode = move_mode

        # Relative displacement or absolute target
        self.move_value = move_value

        self.trajectory_time = trajectory_time

        # =================================================
        # State
        # =================================================

        self.current_positions = {}

        self.initial_positions = None
        self.target_positions = None

        self.q0 = None
        self.qf = None

        self.start_time = None
        self.last_joint_state_time = None

        self.trajectory_started = False
        self.reference_finished = False

        self.done = False
        self.success = False

        self.last_log_time = 0.0

        # =================================================
        # ROS 2 JointState subscriber
        # =================================================

        self.joint_state_sub = \
            self.create_subscription(
                JointState,
                "/joint_states",
                self.joint_state_callback,
                10
            )

        # =================================================
        # ROS 2 JointCommand publisher
        # =================================================

        self.command_pub = \
            self.create_publisher(
                JointCommand,
                "/robot/limb/right/joint_command",
                1
            )

        # =================================================
        # 100 Hz trajectory loop
        # =================================================

        self.timer = \
            self.create_timer(
                1.0 / CONTROL_RATE_HZ,
                self.control_loop
            )

        # =================================================

        self.get_logger().info(
            "Sawyer single-joint trajectory node started"
        )

        self.get_logger().info(
            f"Selected joint: {self.test_joint}"
        )

        self.get_logger().info(
            f"Motion mode: {self.move_mode}"
        )

        self.get_logger().info(
            "Waiting for Sawyer joint states..."
        )

    # =====================================================
    # Joint-state feedback
    # =====================================================

    def joint_state_callback(self, msg):

        self.last_joint_state_time = \
            time.monotonic()

        for name, position in zip(
            msg.name,
            msg.position
        ):

            if name in RIGHT_JOINTS:

                self.current_positions[name] = \
                    float(position)

        # Already initialized.
        if self.trajectory_started:
            return

        # Wait until all seven right-arm joints are known.
        if not all(
            joint in self.current_positions
            for joint in RIGHT_JOINTS
        ):
            return

        self.initialize_trajectory()

    # =====================================================
    # Initialize trajectory
    # =====================================================

    def initialize_trajectory(self):

        self.initial_positions = {

            joint: self.current_positions[joint]

            for joint in RIGHT_JOINTS
        }

        self.target_positions = \
            self.initial_positions.copy()

        # -------------------------------------------------
        # Starting position
        # -------------------------------------------------

        self.q0 = \
            self.initial_positions[
                self.test_joint
            ]

        # -------------------------------------------------
        # Determine final position
        # -------------------------------------------------

        if self.move_mode == "relative":

            self.qf = \
                self.q0 + self.move_value

        elif self.move_mode == "absolute":

            self.qf = \
                self.move_value

        else:

            self.get_logger().error(
                "Invalid motion mode"
            )

            self.done = True
            return

        # -------------------------------------------------
        # Joint-limit check
        # -------------------------------------------------

        limits = \
            JOINT_LIMITS[
                self.test_joint
            ]

        lower = limits["lower"]
        upper = limits["upper"]

        if (
            self.qf < lower
            or self.qf > upper
        ):

            self.get_logger().error(
                "========================================"
            )

            self.get_logger().error(
                "JOINT LIMIT VIOLATION"
            )

            self.get_logger().error(
                f"Joint: {self.test_joint}"
            )

            self.get_logger().error(
                f"Current: {self.q0:.6f} rad"
            )

            self.get_logger().error(
                f"Requested target: {self.qf:.6f} rad"
            )

            self.get_logger().error(
                f"Allowed range: "
                f"[{lower:.6f}, {upper:.6f}] rad"
            )

            self.get_logger().error(
                "Movement cancelled."
            )

            self.get_logger().error(
                "========================================"
            )

            self.done = True

            return

        # -------------------------------------------------
        # Calculate displacement
        # -------------------------------------------------

        displacement = \
            self.qf - self.q0

        # -------------------------------------------------
        # Quintic peak velocity check
        #
        # For:
        #
        # s(tau) =
        # 10 tau^3 - 15 tau^4 + 6 tau^5
        #
        # maximum ds/dtau = 1.875
        #
        # Therefore:
        #
        # vmax = 1.875 * |qf-q0| / T
        # -------------------------------------------------

        expected_peak_velocity = \
            1.875 \
            * abs(displacement) \
            / self.trajectory_time

        velocity_limit = \
            limits["velocity"]

        if expected_peak_velocity > velocity_limit:

            minimum_time = (
                1.875
                * abs(displacement)
                / velocity_limit
            )

            self.get_logger().error(
                "========================================"
            )

            self.get_logger().error(
                "TRAJECTORY TOO FAST"
            )

            self.get_logger().error(
                f"Expected peak velocity: "
                f"{expected_peak_velocity:.3f} rad/s"
            )

            self.get_logger().error(
                f"Joint velocity limit: "
                f"{velocity_limit:.3f} rad/s"
            )

            self.get_logger().error(
                f"Use trajectory time > "
                f"{minimum_time:.3f} s"
            )

            self.get_logger().error(
                "========================================"
            )

            self.done = True

            return

        # -------------------------------------------------
        # Set target
        # -------------------------------------------------

        self.target_positions[
            self.test_joint
        ] = self.qf

        # -------------------------------------------------

        self.start_time = \
            time.monotonic()

        self.trajectory_started = True

        # -------------------------------------------------
        # Information
        # -------------------------------------------------

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "TRAJECTORY INITIALIZED"
        )

        self.get_logger().info(
            f"Joint: {self.test_joint}"
        )

        self.get_logger().info(
            f"Initial position: "
            f"{self.q0:.6f} rad"
        )

        self.get_logger().info(
            f"Target position: "
            f"{self.qf:.6f} rad"
        )

        self.get_logger().info(
            f"Displacement: "
            f"{displacement:.6f} rad"
        )

        self.get_logger().info(
            f"Displacement: "
            f"{math.degrees(displacement):.2f} deg"
        )

        self.get_logger().info(
            f"Trajectory duration: "
            f"{self.trajectory_time:.2f} s"
        )

        self.get_logger().info(
            f"Expected peak velocity: "
            f"{expected_peak_velocity:.3f} rad/s"
        )

        self.get_logger().info(
            f"Joint velocity limit: "
            f"{velocity_limit:.3f} rad/s"
        )

        self.get_logger().info(
            f"Command rate: "
            f"{CONTROL_RATE_HZ:.1f} Hz"
        )

        self.get_logger().info(
            "========================================"
        )

    # =====================================================
    # Quintic time scaling
    # =====================================================

    def quintic_scale(self, tau):

        return (
            10.0 * tau**3
            - 15.0 * tau**4
            + 6.0 * tau**5
        )

    # =====================================================
    # Publish JointCommand
    # =====================================================

    def publish_command(
        self,
        commanded_positions
    ):

        msg = JointCommand()

        msg.header.stamp = \
            self.get_clock().now().to_msg()

        msg.mode = \
            JointCommand.POSITION_MODE

        msg.names = list(
            RIGHT_JOINTS
        )

        msg.position = [

            commanded_positions[joint]

            for joint in RIGHT_JOINTS
        ]

        msg.velocity = []
        msg.acceleration = []
        msg.effort = []

        self.command_pub.publish(
            msg
        )

    # =====================================================
    # Main 100 Hz control loop
    # =====================================================

    def control_loop(self):

        if self.done:
            return

        if not self.trajectory_started:
            return

        # =================================================
        # Check feedback freshness
        # =================================================

        if self.last_joint_state_time is None:

            self.abort(
                "No JointState feedback"
            )

            return

        state_age = (
            time.monotonic()
            - self.last_joint_state_time
        )

        if state_age > JOINT_STATE_TIMEOUT:

            self.abort(
                "JointState feedback timeout"
            )

            return

        # =================================================
        # Current time
        # =================================================

        elapsed = (
            time.monotonic()
            - self.start_time
        )

        # =================================================
        # PHASE 1:
        # Follow quintic reference trajectory
        # =================================================

        if elapsed <= self.trajectory_time:

            tau = (
                elapsed
                / self.trajectory_time
            )

            tau = max(
                0.0,
                min(
                    1.0,
                    tau
                )
            )

            scale = \
                self.quintic_scale(
                    tau
                )

            # ---------------------------------------------
            # Hold every other joint at its initial value.
            # ---------------------------------------------

            commanded_positions = \
                self.initial_positions.copy()

            # ---------------------------------------------
            # Selected joint follows trajectory.
            # ---------------------------------------------

            commanded_position = (
                self.q0
                + (self.qf - self.q0)
                * scale
            )

            commanded_positions[
                self.test_joint
            ] = commanded_position

            # ---------------------------------------------
            # Send through ROS2 -> gateway -> ROS1 -> Sawyer
            # ---------------------------------------------

            self.publish_command(
                commanded_positions
            )

            # ---------------------------------------------
            # Progress output every 0.5 s
            # ---------------------------------------------

            now = \
                time.monotonic()

            if (
                now - self.last_log_time
                >= 0.5
            ):

                self.last_log_time = now

                actual = \
                    self.current_positions[
                        self.test_joint
                    ]

                tracking_error = (
                    commanded_position
                    - actual
                )

                self.get_logger().info(
                    f"t={elapsed:.2f}s "
                    f"progress={scale * 100.0:.1f}% "
                    f"desired={commanded_position:.4f} "
                    f"actual={actual:.4f} "
                    f"error={tracking_error:.4f}"
                )

            return

        # =================================================
        # PHASE 2:
        # Reference finished.
        #
        # Continue sending final target until Sawyer
        # reaches the requested position.
        # =================================================

        self.publish_command(
            self.target_positions
        )

        if not self.reference_finished:

            self.reference_finished = True

            self.get_logger().info(
                "========================================"
            )

            self.get_logger().info(
                "REFERENCE TRAJECTORY COMPLETE"
            )

            self.get_logger().info(
                "Holding final target..."
            )

            self.get_logger().info(
                "========================================"
            )

        # -------------------------------------------------
        # Selected joint final error
        # -------------------------------------------------

        actual = \
            self.current_positions[
                self.test_joint
            ]

        error = abs(
            self.qf - actual
        )

        # -------------------------------------------------
        # Success
        # -------------------------------------------------

        if error <= POSITION_TOLERANCE:

            self.success = True
            self.done = True

            self.get_logger().info(
                "========================================"
            )

            self.get_logger().info(
                "TRAJECTORY SUCCESSFUL"
            )

            self.get_logger().info(
                f"Joint: {self.test_joint}"
            )

            self.get_logger().info(
                f"Target: "
                f"{self.qf:.6f} rad"
            )

            self.get_logger().info(
                f"Actual: "
                f"{actual:.6f} rad"
            )

            self.get_logger().info(
                f"Final error: "
                f"{error:.6f} rad"
            )

            self.get_logger().info(
                f"Final error: "
                f"{math.degrees(error):.3f} deg"
            )

            self.get_logger().info(
                f"Total elapsed time: "
                f"{elapsed:.3f} s"
            )

            self.get_logger().info(
                "========================================"
            )

            return

        # -------------------------------------------------
        # Settling timeout
        # -------------------------------------------------

        if (
            elapsed
            > self.trajectory_time
            + SETTLING_TIMEOUT
        ):

            self.get_logger().error(
                "========================================"
            )

            self.get_logger().error(
                "SETTLING TIMEOUT"
            )

            self.get_logger().error(
                f"Joint: {self.test_joint}"
            )

            self.get_logger().error(
                f"Target: {self.qf:.6f}"
            )

            self.get_logger().error(
                f"Actual: {actual:.6f}"
            )

            self.get_logger().error(
                f"Error: {error:.6f}"
            )

            self.get_logger().error(
                "========================================"
            )

            self.done = True
            self.success = False

    # =====================================================
    # Abort helper
    # =====================================================

    def abort(self, reason):

        self.done = True
        self.success = False

        self.get_logger().error(
            "========================================"
        )

        self.get_logger().error(
            "TRAJECTORY ABORTED"
        )

        self.get_logger().error(
            reason
        )

        self.get_logger().error(
            "========================================"
        )


# =========================================================
# Command-line arguments
# =========================================================

def parse_arguments():

    parser = argparse.ArgumentParser(
        description=(
            "Move one Sawyer joint using a smooth "
            "quintic trajectory."
        )
    )

    parser.add_argument(
        "--joint",
        required=True,
        choices=RIGHT_JOINTS,
        help="Sawyer joint to move"
    )

    # -----------------------------------------------------
    # User must choose EXACTLY ONE:
    #
    # --relative
    #
    # or
    #
    # --absolute
    # -----------------------------------------------------

    target_group = \
        parser.add_mutually_exclusive_group(
            required=True
        )

    target_group.add_argument(
        "--relative",
        type=float,
        help=(
            "Relative movement in radians. "
            "Example: 1.5708 for +pi/2"
        )
    )

    target_group.add_argument(
        "--absolute",
        type=float,
        help=(
            "Absolute target joint position "
            "in radians."
        )
    )

    parser.add_argument(
        "--time",
        type=float,
        default=5.0,
        help=(
            "Trajectory duration in seconds "
            "(default: 5.0)"
        )
    )

    # parse_known_args allows ROS 2 arguments to remain.
    args, ros_args = \
        parser.parse_known_args()

    if args.time <= 0.0:

        parser.error(
            "--time must be greater than zero"
        )

    if args.relative is not None:

        move_mode = "relative"
        move_value = args.relative

    else:

        move_mode = "absolute"
        move_value = args.absolute

    return (
        args,
        ros_args,
        move_mode,
        move_value
    )


# =========================================================
# Main
# =========================================================

def main():

    (
        args,
        ros_args,
        move_mode,
        move_value
    ) = parse_arguments()

    rclpy.init(
        args=ros_args
    )

    node = \
        SawyerSingleJointTrajectory(
            test_joint=args.joint,
            move_mode=move_mode,
            move_value=move_value,
            trajectory_time=args.time
        )

    try:

        while (
            rclpy.ok()
            and not node.done
        ):

            rclpy.spin_once(
                node,
                timeout_sec=0.001
            )

    except KeyboardInterrupt:

        node.get_logger().warning(
            "Trajectory interrupted by user"
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
