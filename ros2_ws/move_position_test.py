#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from intera_core_msgs.msg import JointCommand


# =========================================================
# Sawyer joints
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
# Test configuration
# =========================================================

# Move only this joint.
TEST_JOINT = "right_j0"

# Small test movement.
#
# 0.02 rad ~= 1.15 degrees
DELTA_RAD = 0.02

# Match Intera move_to_joint_positions()
CONTROL_RATE_HZ = 100.0

# Intera default tolerance:
# 0.008726646 rad ~= 0.5 degrees
POSITION_TOLERANCE = 0.008726646

# Do not command indefinitely.
MOVE_TIMEOUT_SEC = 5.0


class SawyerPositionMoveTest(Node):

    def __init__(self):

        super().__init__(
            "sawyer_position_move_test"
        )

        # -------------------------------------------------
        # Current Sawyer state
        # -------------------------------------------------

        self.current_positions = {}

        # Target will be generated from the first complete
        # JointState received.
        self.target_positions = None

        self.move_start_time = None

        self.done = False
        self.success = False

        self.last_log_time = 0.0

        # -------------------------------------------------
        # Joint state subscriber
        # -------------------------------------------------

        self.joint_state_sub = \
            self.create_subscription(
                JointState,
                "/joint_states",
                self.joint_state_callback,
                10
            )

        # -------------------------------------------------
        # Joint command publisher
        # -------------------------------------------------

        self.command_pub = \
            self.create_publisher(
                JointCommand,
                "/robot/limb/right/joint_command",
                1
            )

        # -------------------------------------------------
        # 100 Hz control loop
        # -------------------------------------------------

        self.control_timer = \
            self.create_timer(
                1.0 / CONTROL_RATE_HZ,
                self.control_loop
            )

        self.get_logger().info(
            "Sawyer position move test started"
        )

        self.get_logger().info(
            f"Waiting for all {len(RIGHT_JOINTS)} "
            "right-arm joint states..."
        )

    # =====================================================
    # Joint state feedback
    # =====================================================

    def joint_state_callback(self, msg):

        for name, position in zip(
            msg.name,
            msg.position
        ):

            if name in RIGHT_JOINTS:

                self.current_positions[name] = \
                    float(position)

        # -------------------------------------------------
        # Create target only once.
        # -------------------------------------------------

        if self.target_positions is not None:
            return

        # Wait until all seven joints are available.
        if not all(
            joint in self.current_positions
            for joint in RIGHT_JOINTS
        ):
            return

        # -------------------------------------------------
        # Current configuration becomes our baseline.
        # -------------------------------------------------

        self.target_positions = {
            joint: self.current_positions[joint]
            for joint in RIGHT_JOINTS
        }

        initial_position = \
            self.current_positions[
                TEST_JOINT
            ]

        target_position = \
            initial_position + DELTA_RAD

        # Change ONLY right_j0.
        self.target_positions[
            TEST_JOINT
        ] = target_position

        self.move_start_time = \
            time.monotonic()

        # -------------------------------------------------
        # Print exact experiment
        # -------------------------------------------------

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "Target initialized"
        )

        self.get_logger().info(
            f"{TEST_JOINT} initial = "
            f"{initial_position:.6f} rad"
        )

        self.get_logger().info(
            f"{TEST_JOINT} target  = "
            f"{target_position:.6f} rad"
        )

        self.get_logger().info(
            f"Requested change = "
            f"{DELTA_RAD:.6f} rad"
        )

        self.get_logger().info(
            f"Tolerance = "
            f"{POSITION_TOLERANCE:.6f} rad"
        )

        self.get_logger().info(
            f"Publishing at "
            f"{CONTROL_RATE_HZ:.1f} Hz"
        )

        self.get_logger().info(
            "========================================"
        )

    # =====================================================
    # 100 Hz control loop
    # =====================================================

    def control_loop(self):

        if self.done:
            return

        # Target has not been initialized yet.
        if self.target_positions is None:
            return

        # -------------------------------------------------
        # Make sure we still have all joint feedback.
        # -------------------------------------------------

        if not all(
            joint in self.current_positions
            for joint in RIGHT_JOINTS
        ):

            return

        # -------------------------------------------------
        # Calculate joint errors.
        # -------------------------------------------------

        errors = {}

        for joint in RIGHT_JOINTS:

            target = \
                self.target_positions[
                    joint
                ]

            current = \
                self.current_positions[
                    joint
                ]

            errors[joint] = abs(
                target - current
            )

        max_error_joint = max(
            errors,
            key=errors.get
        )

        max_error = \
            errors[
                max_error_joint
            ]

        # -------------------------------------------------
        # Target reached
        # -------------------------------------------------

        if max_error < POSITION_TOLERANCE:

            elapsed = (
                time.monotonic()
                - self.move_start_time
            )

            self.get_logger().info(
                "========================================"
            )

            self.get_logger().info(
                "TARGET REACHED"
            )

            self.get_logger().info(
                f"Elapsed time: "
                f"{elapsed:.3f} s"
            )

            self.get_logger().info(
                f"Maximum final error: "
                f"{max_error:.6f} rad"
            )

            self.get_logger().info(
                f"Worst joint: "
                f"{max_error_joint}"
            )

            self.get_logger().info(
                f"{TEST_JOINT} final = "
                f"{self.current_positions[TEST_JOINT]:.6f}"
            )

            self.get_logger().info(
                "========================================"
            )

            self.success = True
            self.done = True

            return

        # -------------------------------------------------
        # Timeout
        # -------------------------------------------------

        elapsed = (
            time.monotonic()
            - self.move_start_time
        )

        if elapsed > MOVE_TIMEOUT_SEC:

            self.get_logger().error(
                "========================================"
            )

            self.get_logger().error(
                "MOVE TIMEOUT"
            )

            self.get_logger().error(
                f"Elapsed: {elapsed:.3f} s"
            )

            self.get_logger().error(
                f"Maximum error: "
                f"{max_error:.6f} rad"
            )

            self.get_logger().error(
                f"Worst joint: "
                f"{max_error_joint}"
            )

            self.get_logger().error(
                "========================================"
            )

            self.done = True
            self.success = False

            return

        # -------------------------------------------------
        # Construct JointCommand
        # -------------------------------------------------

        command = JointCommand()

        command.header.stamp = \
            self.get_clock().now().to_msg()

        command.mode = \
            JointCommand.POSITION_MODE

        command.names = list(
            RIGHT_JOINTS
        )

        command.position = [
            self.target_positions[joint]
            for joint in RIGHT_JOINTS
        ]

        command.velocity = []
        command.acceleration = []
        command.effort = []

        # -------------------------------------------------
        # Send command
        # -------------------------------------------------

        self.command_pub.publish(
            command
        )

        # -------------------------------------------------
        # Print progress about twice per second.
        #
        # Do NOT print at 100 Hz.
        # -------------------------------------------------

        now = time.monotonic()

        if (
            now - self.last_log_time
            >= 0.5
        ):

            self.last_log_time = now

            current = \
                self.current_positions[
                    TEST_JOINT
                ]

            target = \
                self.target_positions[
                    TEST_JOINT
                ]

            error = \
                abs(
                    target - current
                )

            self.get_logger().info(
                f"{TEST_JOINT}: "
                f"current={current:.6f}, "
                f"target={target:.6f}, "
                f"error={error:.6f}"
            )


def main(args=None):

    rclpy.init(
        args=args
    )

    node = \
        SawyerPositionMoveTest()

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
            "Movement test interrupted by user"
        )

    finally:

        node.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


if __name__ == "__main__":

    main()
