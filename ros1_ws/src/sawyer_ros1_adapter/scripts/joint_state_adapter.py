#!/usr/bin/env python3

import json
import socket
import threading

import rospy

from sensor_msgs.msg import JointState

from intera_core_msgs.msg import (
    EndpointState,
    RobotAssemblyState,
)


HOST = "127.0.0.1"
PORT = 5005


class SawyerROS1Adapter:

    def __init__(self):

        self.connection = None
        self.connection_lock = threading.Lock()

        # -------------------------------------------------
        # TCP server
        # -------------------------------------------------

        self.server_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(1)

        rospy.loginfo(
            "ROS 1 adapter listening on %s:%d",
            HOST,
            PORT
        )

        # Accept gateway connections in background.
        self.accept_thread = threading.Thread(
            target=self.accept_connections,
            daemon=True
        )

        self.accept_thread.start()

        # -------------------------------------------------
        # Sawyer ROS 1 subscriptions
        # -------------------------------------------------

        rospy.Subscriber(
            "/robot/joint_states",
            JointState,
            self.joint_state_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.Subscriber(
            "/robot/limb/right/endpoint_state",
            EndpointState,
            self.endpoint_state_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.Subscriber(
            "/robot/state",
            RobotAssemblyState,
            self.robot_state_callback,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.loginfo(
            "Subscribed to Sawyer read-only state topics"
        )

    # =====================================================
    # IPC
    # =====================================================

    def accept_connections(self):

        while not rospy.is_shutdown():

            try:

                connection, address = \
                    self.server_socket.accept()

                rospy.loginfo(
                    "ROS 2 gateway connected: %s",
                    address
                )

                with self.connection_lock:

                    if self.connection is not None:

                        try:
                            self.connection.close()
                        except Exception:
                            pass

                    self.connection = connection

            except OSError:

                if not rospy.is_shutdown():

                    rospy.logwarn(
                        "IPC accept error"
                    )

    def send_packet(self, packet):

        data = (
            json.dumps(
                packet,
                separators=(",", ":")
            )
            + "\n"
        ).encode("utf-8")

        with self.connection_lock:

            if self.connection is None:
                return

            try:

                self.connection.sendall(data)

            except (BrokenPipeError, ConnectionResetError, OSError) \
                    as error:

                rospy.logwarn(
                    "ROS 2 gateway disconnected: %s",
                    error
                )

                try:
                    self.connection.close()
                except Exception:
                    pass

                self.connection = None

    # =====================================================
    # /robot/joint_states
    # =====================================================

    def joint_state_callback(self, msg):

        packet = {

            "type": "joint_state",

            "header": {
                "sec": msg.header.stamp.secs,
                "nanosec": msg.header.stamp.nsecs,
                "frame_id": msg.header.frame_id
            },

            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort)
        }

        self.send_packet(packet)

    # =====================================================
    # /robot/limb/right/endpoint_state
    # =====================================================

    def endpoint_state_callback(self, msg):

        packet = {

            "type": "endpoint_state",

            "header": {
                "sec": msg.header.stamp.secs,
                "nanosec": msg.header.stamp.nsecs,
                "frame_id": msg.header.frame_id
            },

            "pose": {

                "position": {
                    "x": msg.pose.position.x,
                    "y": msg.pose.position.y,
                    "z": msg.pose.position.z
                },

                "orientation": {
                    "x": msg.pose.orientation.x,
                    "y": msg.pose.orientation.y,
                    "z": msg.pose.orientation.z,
                    "w": msg.pose.orientation.w
                }
            },

            "twist": {

                "linear": {
                    "x": msg.twist.linear.x,
                    "y": msg.twist.linear.y,
                    "z": msg.twist.linear.z
                },

                "angular": {
                    "x": msg.twist.angular.x,
                    "y": msg.twist.angular.y,
                    "z": msg.twist.angular.z
                }
            },

            "wrench": {

                "force": {
                    "x": msg.wrench.force.x,
                    "y": msg.wrench.force.y,
                    "z": msg.wrench.force.z
                },

                "torque": {
                    "x": msg.wrench.torque.x,
                    "y": msg.wrench.torque.y,
                    "z": msg.wrench.torque.z
                }
            },

            "valid": msg.valid
        }

        self.send_packet(packet)

    # =====================================================
    # /robot/state
    # =====================================================

    def robot_state_callback(self, msg):

        packet = {

            "type": "robot_state",

            "homed": msg.homed,
            "ready": msg.ready,
            "enabled": msg.enabled,
            "stopped": msg.stopped,
            "error": msg.error,

            # ROS 1 name -> ROS 2 name mapping
            "low_voltage": msg.lowVoltage,

            "estop_button": int(msg.estop_button),
            "estop_source": int(msg.estop_source)
        }

        self.send_packet(packet)

    # =====================================================

    def shutdown(self):

        with self.connection_lock:

            if self.connection is not None:

                try:
                    self.connection.close()
                except Exception:
                    pass

        try:
            self.server_socket.close()
        except Exception:
            pass


def main():

    rospy.init_node(
        "sawyer_ros1_adapter"
    )

    adapter = SawyerROS1Adapter()

    rospy.on_shutdown(
        adapter.shutdown
    )

    rospy.spin()


if __name__ == "__main__":
    main()
