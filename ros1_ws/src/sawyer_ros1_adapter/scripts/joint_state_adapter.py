#!/usr/bin/env python3

import json
import socket

import rospy
from sensor_msgs.msg import JointState


HOST = "127.0.0.1"
PORT = 5005


def main():

    rospy.init_node("sawyer_joint_state_adapter")

    # Create TCP server
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server_socket.setsockopt(
        socket.SOL_SOCKET,
        socket.SO_REUSEADDR,
        1
    )

    server_socket.bind((HOST, PORT))

    server_socket.listen(1)

    rospy.loginfo(
        "Waiting for ROS 2 gateway on %s:%d...",
        HOST,
        PORT
    )

    connection, address = server_socket.accept()

    rospy.loginfo(
        "ROS 2 gateway connected: %s",
        address
    )

    def joint_state_callback(msg):

        data = {
            "stamp_sec": msg.header.stamp.secs,
            "stamp_nsec": msg.header.stamp.nsecs,

            "name": list(msg.name),
            "position": list(msg.position),
            "velocity": list(msg.velocity),
            "effort": list(msg.effort)
        }

        message = json.dumps(data) + "\n"

        try:
            connection.sendall(
                message.encode("utf-8")
            )

        except socket.error as error:

            rospy.logerr(
                "IPC connection error: %s",
                error
            )

            rospy.signal_shutdown(
                "ROS 2 gateway disconnected"
            )

    rospy.Subscriber(
        "/robot/joint_states",
        JointState,
        joint_state_callback,
        queue_size=1,
        tcp_nodelay=True
    )

    rospy.loginfo(
        "Subscribed to /robot/joint_states"
    )

    rospy.spin()

    connection.close()
    server_socket.close()


if __name__ == "__main__":
    main()
