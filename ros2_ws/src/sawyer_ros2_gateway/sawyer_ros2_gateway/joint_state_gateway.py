#!/usr/bin/env python3

import json
import socket

import rclpy

from rclpy.node import Node

from sensor_msgs.msg import JointState

from intera_core_msgs.msg import (
    EndpointState,
    RobotAssemblyState,
)


HOST = "127.0.0.1"
PORT = 5005


class SawyerROS2Gateway(Node):

    def __init__(self):

        super().__init__(
            "sawyer_ros2_gateway"
        )

        # ===============================================
        # ROS 2 publishers
        # ===============================================

        self.joint_state_pub = \
            self.create_publisher(
                JointState,
                "/joint_states",
                10
            )

        self.endpoint_state_pub = \
            self.create_publisher(
                EndpointState,
                "/robot/limb/right/endpoint_state",
                10
            )

        self.robot_state_pub = \
            self.create_publisher(
                RobotAssemblyState,
                "/robot/state",
                10
            )

        # ===============================================
        # IPC state
        # ===============================================

        self.socket = None

        self.receive_buffer = b""

        self.connection_message_printed = False

        # Poll IPC frequently.
        self.timer = self.create_timer(
            0.005,
            self.poll_socket
        )

        self.get_logger().info(
            "Sawyer ROS 2 gateway started"
        )

    # ===================================================
    # Connection
    # ===================================================

    def connect(self):

        try:

            sock = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            sock.settimeout(0.2)

            sock.connect(
                (HOST, PORT)
            )

            sock.setblocking(False)

            self.socket = sock
            self.receive_buffer = b""

            self.get_logger().info(
                f"Connected to ROS 1 adapter at "
                f"{HOST}:{PORT}"
            )

        except (
            ConnectionRefusedError,
            socket.timeout,
            OSError
        ):

            try:
                sock.close()
            except Exception:
                pass

            self.socket = None

    # ===================================================
    # Receive IPC
    # ===================================================

    def poll_socket(self):

        if self.socket is None:

            self.connect()

            return

        try:

            data = self.socket.recv(16384)

            if not data:

                self.handle_disconnect()

                return

            self.receive_buffer += data

            while b"\n" in self.receive_buffer:

                line, self.receive_buffer = \
                    self.receive_buffer.split(
                        b"\n",
                        1
                    )

                if not line:
                    continue

                self.process_packet(line)

        except BlockingIOError:

            pass

        except (
            ConnectionResetError,
            OSError
        ) as error:

            self.get_logger().warning(
                f"IPC connection lost: {error}"
            )

            self.handle_disconnect()

    def handle_disconnect(self):

        self.get_logger().warning(
            "ROS 1 adapter disconnected"
        )

        if self.socket is not None:

            try:
                self.socket.close()
            except Exception:
                pass

        self.socket = None
        self.receive_buffer = b""

    # ===================================================
    # Packet dispatcher
    # ===================================================

    def process_packet(self, line):

        try:

            packet = json.loads(
                line.decode("utf-8")
            )

            packet_type = packet.get(
                "type"
            )

            if packet_type == "joint_state":

                self.publish_joint_state(
                    packet
                )

            elif packet_type == "endpoint_state":

                self.publish_endpoint_state(
                    packet
                )

            elif packet_type == "robot_state":

                self.publish_robot_state(
                    packet
                )

            else:

                self.get_logger().warning(
                    f"Unknown IPC packet type: "
                    f"{packet_type}"
                )

        except Exception as error:

            self.get_logger().error(
                f"Failed to process IPC packet: "
                f"{error}"
            )

    # ===================================================
    # Joint State
    # ===================================================

    def publish_joint_state(self, data):

        msg = JointState()

        msg.header.stamp.sec = \
            int(data["header"]["sec"])

        msg.header.stamp.nanosec = \
            int(data["header"]["nanosec"])

        msg.header.frame_id = \
            data["header"]["frame_id"]

        msg.name = data["name"]

        msg.position = data["position"]

        msg.velocity = data["velocity"]

        msg.effort = data["effort"]

        self.joint_state_pub.publish(
            msg
        )

    # ===================================================
    # Endpoint State
    # ===================================================

    def publish_endpoint_state(self, data):

        msg = EndpointState()

        # Header
        msg.header.stamp.sec = \
            int(data["header"]["sec"])

        msg.header.stamp.nanosec = \
            int(data["header"]["nanosec"])

        msg.header.frame_id = \
            data["header"]["frame_id"]

        # Position
        msg.pose.position.x = \
            data["pose"]["position"]["x"]

        msg.pose.position.y = \
            data["pose"]["position"]["y"]

        msg.pose.position.z = \
            data["pose"]["position"]["z"]

        # Orientation
        msg.pose.orientation.x = \
            data["pose"]["orientation"]["x"]

        msg.pose.orientation.y = \
            data["pose"]["orientation"]["y"]

        msg.pose.orientation.z = \
            data["pose"]["orientation"]["z"]

        msg.pose.orientation.w = \
            data["pose"]["orientation"]["w"]

        # Linear velocity
        msg.twist.linear.x = \
            data["twist"]["linear"]["x"]

        msg.twist.linear.y = \
            data["twist"]["linear"]["y"]

        msg.twist.linear.z = \
            data["twist"]["linear"]["z"]

        # Angular velocity
        msg.twist.angular.x = \
            data["twist"]["angular"]["x"]

        msg.twist.angular.y = \
            data["twist"]["angular"]["y"]

        msg.twist.angular.z = \
            data["twist"]["angular"]["z"]

        # Force
        msg.wrench.force.x = \
            data["wrench"]["force"]["x"]

        msg.wrench.force.y = \
            data["wrench"]["force"]["y"]

        msg.wrench.force.z = \
            data["wrench"]["force"]["z"]

        # Torque
        msg.wrench.torque.x = \
            data["wrench"]["torque"]["x"]

        msg.wrench.torque.y = \
            data["wrench"]["torque"]["y"]

        msg.wrench.torque.z = \
            data["wrench"]["torque"]["z"]

        msg.valid = data["valid"]

        self.endpoint_state_pub.publish(
            msg
        )

    # ===================================================
    # Robot State
    # ===================================================

    def publish_robot_state(self, data):

        msg = RobotAssemblyState()

        msg.homed = data["homed"]
        msg.ready = data["ready"]
        msg.enabled = data["enabled"]
        msg.stopped = data["stopped"]
        msg.error = data["error"]

        # ROS 2 migrated field name
        msg.low_voltage = \
            data["low_voltage"]

        msg.estop_button = \
            int(data["estop_button"])

        msg.estop_source = \
            int(data["estop_source"])

        self.robot_state_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = SawyerROS2Gateway()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    if node.socket is not None:

        try:
            node.socket.close()
        except Exception:
            pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()