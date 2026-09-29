#!/usr/bin/env python3

import math
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
# Trajectory configuration
# =========================================================

TEST_JOINT = "right_j0"

# Move +pi/2 rad = +90 degrees
DELTA_POSITION = math.pi / 2.0

# Duration of trajectory
TRAJECTORY_TIME = 5.0

# Command frequency
CONTROL_RATE_HZ = 100.0

# After trajectory generation finishes, keep publishing
# the final target until the physical joint reaches it.
POSITION_TOLERANCE = 0.01

# Maximum additional time allowed after trajectory
SETTLING_TIMEOUT = 3.0


# right_j0 approximate joint limits from Sawyer description
J0_LOWER_LIMIT = -3.0503
J0_UPPER_LIMIT = 3.0503


class SawyerJointTrajectory(Node):

    def __init__(self):

        super().__init__(
            "sawyer_joint_trajectory"
        )

        # -------------------------------------------------
        # State
        # -------------------------------------------------

        self.current_positions = {}

        self.initial_positions = None
        self.target_positions = None

        self.start_time = None

        self.trajectory_started = False
        self.trajectory_finished = False
        self.done = False

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

        self.timer = \
            self.create_timer(
                1.0 / CONTROL_RATE_HZ,
                self.control_loop
            )

        self.get_logger().info(
            "Sawyer trajectory node started"
        )

        self.get_logger().info(
            "Waiting for Sawyer joint states..."
        )

    # =====================================================
    # Joint-state feedback
    # =====================================================

    def joint_state_callback(self, msg):

        for name, position in zip(
            msg.name,
            msg.position
        ):

            if name in RIGHT_JOINTS:

                self.current_positions[name] = \
                    float(position)

        # Wait until all seven joints are available.
        if self.trajectory_started:
            return

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

        q0 = \
            self.initial_positions[
                TEST_JOINT
            ]

        qf = \
            q0 + DELTA_POSITION

        # -------------------------------------------------
        # Joint-limit safety check
        # -------------------------------------------------

        if (
            qf < J0_LOWER_LIMIT
            or qf > J0_UPPER_LIMIT
        ):

            self.get_logger().error(
                "Requested trajectory exceeds "
                "right_j0 joint limits"
            )

            self.get_logger().error(
                f"Initial: {q0:.4f} rad"
            )

            self.get_logger().error(
                f"Requested target: {qf:.4f} rad"
            )

            self.get_logger().error(
                f"Allowed range: "
                f"[{J0_LOWER_LIMIT:.4f}, "
                f"{J0_UPPER_LIMIT:.4f}]"
            )

            self.done = True
            return

        self.target_positions[
            TEST_JOINT
        ] = qf

        self.start_time = \
            time.monotonic()

        self.trajectory_started = True

        self.get_logger().info(
            "========================================"
        )

        self.get_logger().info(
            "TRAJECTORY INITIALIZED"
        )

        self.get_logger().info(
            f"Joint: {TEST_JOINT}"
        )

        self.get_logger().info(
            f"Initial position: "
            f"{q0:.6f} rad"
        )

        self.get_logger().info(
            f"Target position: "
            f"{qf:.6f} rad"
        )

        self.get_logger().info(
            f"Movement: "
            f"{DELTA_POSITION:.6f} rad"
        )

        self.get_logger().info(
            "Movement: 90 degrees"
        )

        self.get_logger().info(
            f"Trajectory duration: "
            f"{TRAJECTORY_TIME:.2f} s"
        )

        self.get_logger().info(
            f"Command rate: "
            f"{CONTROL_RATE_HZ:.1f} Hz"
        )

        self.get_logger().info(
            "========================================"
        )

    # =====================================================
    # Quintic trajectory
    # =====================================================

    def quintic_scale(self, tau):

        # tau must be between 0 and 1.
        #
        # s(tau) =
        # 10 tau^3 - 15 tau^4 + 6 tau^5
        #
        # Gives:
        #
        # s(0) = 0
        # s(1) = 1
        #
        # velocity = 0 at start/end
        # acceleration = 0 at start/end

        return (
            10.0 * tau**3
            - 15.0 * tau**4
            + 6.0 * tau**5
        )

    # =====================================================
    # Publish position command
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
    # Main trajectory loop
    # =====================================================

    def control_loop(self):

        if self.done:
            return

        if not self.trajectory_started:
            return

        if not all(
            joint in self.current_positions
            for joint in RIGHT_JOINTS
        ):
            return

        elapsed = \
            time.monotonic() \
            - self.start_time

        # =================================================
        # PHASE 1:
        # Generate quintic trajectory
        # =================================================

        if elapsed <= TRAJECTORY_TIME:

            tau = \
                elapsed / TRAJECTORY_TIME

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

            commanded_positions = \
                self.initial_positions.copy()

            q0 = \
                self.initial_positions[
                    TEST_JOINT
                ]

            commanded_position = \
                q0 \
                + DELTA_POSITION * scale

            commanded_positions[
                TEST_JOINT
            ] = commanded_position

            self.publish_command(
                commanded_positions
            )

            # ---------------------------------------------
            # Print progress every 0.5 s
            # ---------------------------------------------

            now = time.monotonic()

            if (
                now - self.last_log_time
                >= 0.5
            ):

                self.last_log_time = now

                actual = \
                    self.current_positions[
                        TEST_JOINT
                    ]

                error = \
                    commanded_position \
                    - actual

                percentage = \
                    scale * 100.0

                self.get_logger().info(
                    f"t={elapsed:.2f}s "
                    f"trajectory={percentage:.1f}% "
                    f"desired={commanded_position:.4f} "
                    f"actual={actual:.4f} "
                    f"error={error:.4f}"
                )

            return

        # =================================================
        # PHASE 2:
        # Trajectory finished.
        #
        # Hold final position until physical robot
        # reaches the target.
        # =================================================

        self.publish_command(
            self.target_positions
        )

        if not self.trajectory_finished:

            self.trajectory_finished = True

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

        actual = \
            self.current_positions[
                TEST_JOINT
            ]

        target = \
            self.target_positions[
                TEST_JOINT
            ]

        error = abs(
            target - actual
        )

        # -------------------------------------------------
        # Target reached
        # -------------------------------------------------

        if error <= POSITION_TOLERANCE:

            self.get_logger().info(
                "========================================"
            )

            self.get_logger().info(
                "TRAJECTORY SUCCESSFUL"
            )

            self.get_logger().info(
                f"Target: "
                f"{target:.6f} rad"
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
                f"Total elapsed time: "
                f"{elapsed:.3f} s"
            )

            self.get_logger().info(
                "========================================"
            )

            self.done = True

            return

        # -------------------------------------------------
        # Settling timeout
        # -------------------------------------------------

        if (
            elapsed
            > TRAJECTORY_TIME
            + SETTLING_TIMEOUT
        ):

            self.get_logger().error(
                "========================================"
            )

            self.get_logger().error(
                "TRAJECTORY SETTLING TIMEOUT"
            )

            self.get_logger().error(
                f"Target: "
                f"{target:.6f}"
            )

            self.get_logger().error(
                f"Actual: "
                f"{actual:.6f}"
            )

            self.get_logger().error(
                f"Error: "
                f"{error:.6f}"
            )

            self.get_logger().error(
                "========================================"
            )

            self.done = True


# =========================================================
# Main
# =========================================================

def main(args=None):

    rclpy.init(
        args=args
    )

    node = \
        SawyerJointTrajectory()

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
