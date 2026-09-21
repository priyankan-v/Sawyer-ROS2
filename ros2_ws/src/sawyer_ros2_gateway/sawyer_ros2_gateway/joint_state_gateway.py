#!/usr/bin/env python3

import json
import socket

import rclpy

from rclpy.node import Node
from sensor_msgs.msg import JointState


HOST = "127.0.0.1"
PORT = 5005


class JointStateGateway(Node):

    def __init__(self):

        super().__init__("sawyer_joint_state_gateway")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        self.socket = None

        self.receive_buffer = b""

        self.get_logger().info(
            "Sawyer ROS 2 joint-state gateway started"
        )

        # Check socket every 5 ms
        self.timer = self.create_timer(
            0.005,
            self.poll_socket
        )

    def connect(self):

        try:

            self.socket = socket.socket(
                socket.AF_INET,
                socket.SOCK_STREAM
            )

            self.socket.settimeout(0.2)

            self.socket.connect(
                (HOST, PORT)
            )

            self.socket.setblocking(False)

            self.get_logger().info(
                f"Connected to ROS 1 adapter at "
                f"{HOST}:{PORT}"
            )

        except (ConnectionRefusedError, socket.timeout):

            if self.socket is not None:
                self.socket.close()

            self.socket = None

    def poll_socket(self):

        if self.socket is None:

            self.connect()

            return

        try:

            data = self.socket.recv(8192)

            if not data:

                self.get_logger().warning(
                    "ROS 1 adapter disconnected"
                )

                self.socket.close()

                self.socket = None

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

                self.process_message(line)

        except BlockingIOError:

            pass

        except socket.error as error:

            self.get_logger().warning(
                f"Socket error: {error}"
            )

            self.socket.close()

            self.socket = None

    def process_message(self, line):

        try:

            data = json.loads(
                line.decode("utf-8")
            )

            msg = JointState()

            msg.header.stamp.sec = \
                data["stamp_sec"]

            msg.header.stamp.nanosec = \
                data["stamp_nsec"]

            msg.name = data["name"]

            msg.position = data["position"]

            msg.velocity = data["velocity"]

            msg.effort = data["effort"]

            self.publisher.publish(msg)

        except Exception as error:

            self.get_logger().error(
                f"Failed to process joint state: {error}"
            )


def main(args=None):

    rclpy.init(args=args)

    node = JointStateGateway()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()