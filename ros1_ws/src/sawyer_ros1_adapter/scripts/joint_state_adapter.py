#!/usr/bin/env python3

import json
import math
import socket
import threading

import rospy

from sensor_msgs.msg import JointState

from intera_core_msgs.msg import (
    EndpointState,
    RobotAssemblyState,
    JointCommand,
)


# =========================================================
# IPC configuration
# =========================================================

HOST = "127.0.0.1"
PORT = 5005


# =========================================================
# Safety configuration
# =========================================================

# KEEP FALSE during the first ROS 2 -> ROS 1 command tests.
#
# False:
#   ROS 2 command -> IPC -> ROS 1 adapter -> validate -> print
#
# True:
#   ROS 2 command -> IPC -> ROS 1 adapter -> Sawyer
#
ENABLE_ROBOT_OUTPUT = True


VALID_RIGHT_JOINTS = {
    "right_j0",
    "right_j1",
    "right_j2",
    "right_j3",
    "right_j4",
    "right_j5",
    "right_j6",
}


class SawyerROS1Adapter:

    def __init__(self):

        # =================================================
        # IPC state
        # =================================================

        self.connection = None
        self.connection_lock = threading.Lock()

        # =================================================
        # Sawyer robot state
        #
        # Used later as safety gates before forwarding
        # commands to the physical robot.
        # =================================================

        self.robot_homed = False
        self.robot_ready = False
        self.robot_enabled = False
        self.robot_stopped = True
        self.robot_error = False

        # =================================================
        # TCP server
        # =================================================

        self.server_socket = socket.socket(
            socket.AF_INET,
            socket.SOCK_STREAM
        )

        self.server_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_REUSEADDR,
            1
        )

        self.server_socket.bind(
            (HOST, PORT)
        )

        self.server_socket.listen(1)

        rospy.loginfo(
            "ROS 1 adapter listening on %s:%d",
            HOST,
            PORT
        )

        # Accept ROS 2 gateway connections in background.
        self.accept_thread = threading.Thread(
            target=self.accept_connections,
            daemon=True
        )

        self.accept_thread.start()

        # =================================================
        # Sawyer ROS 1 state subscriptions
        #
        # Direction:
        #
        # Sawyer -> ROS 1 adapter -> IPC -> ROS 2 gateway
        # =================================================

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

        # =================================================
        # Sawyer ROS 1 command publisher
        #
        # Direction:
        #
        # ROS 2 -> gateway -> IPC -> ROS 1 adapter -> Sawyer
        #
        # Note:
        # Publishing is blocked while
        # ENABLE_ROBOT_OUTPUT == False.
        # =================================================

        self.joint_command_pub = rospy.Publisher(
            "/robot/limb/right/joint_command",
            JointCommand,
            queue_size=1,
            tcp_nodelay=True
        )

        rospy.loginfo(
            "ROS 1 JointCommand publisher created "
            "(robot output enabled: %s)",
            ENABLE_ROBOT_OUTPUT
        )

    # =====================================================
    # IPC - accept ROS 2 gateway connection
    # =====================================================

    def accept_connections(self):

        while not rospy.is_shutdown():

            try:

                connection, address = \
                    self.server_socket.accept()

                # Reduce TCP buffering latency.
                connection.setsockopt(
                    socket.IPPROTO_TCP,
                    socket.TCP_NODELAY,
                    1
                )

                rospy.loginfo(
                    "ROS 2 gateway connected: %s",
                    address
                )

                # -----------------------------------------
                # Replace previous connection
                # -----------------------------------------

                with self.connection_lock:

                    if self.connection is not None:

                        try:
                            self.connection.close()
                        except Exception:
                            pass

                    self.connection = connection

                # -----------------------------------------
                # Start receiver for ROS 2 -> ROS 1 data
                # -----------------------------------------

                receiver_thread = threading.Thread(
                    target=self.receive_packets,
                    args=(connection,),
                    daemon=True
                )

                receiver_thread.start()

            except OSError:

                if not rospy.is_shutdown():

                    rospy.logwarn(
                        "IPC accept error"
                    )

    # =====================================================
    # IPC - ROS 1 -> ROS 2
    # =====================================================

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

                self.connection.sendall(
                    data
                )

            except (
                BrokenPipeError,
                ConnectionResetError,
                OSError
            ) as error:

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
    # IPC - ROS 2 -> ROS 1
    # =====================================================

    def receive_packets(self, connection):

        receive_buffer = b""

        try:

            while not rospy.is_shutdown():

                data = connection.recv(
                    16384
                )

                # Empty recv means peer closed socket.
                if not data:
                    break

                receive_buffer += data

                # -----------------------------------------
                # Packets are newline-delimited JSON
                # -----------------------------------------

                while b"\n" in receive_buffer:

                    line, receive_buffer = \
                        receive_buffer.split(
                            b"\n",
                            1
                        )

                    if not line:
                        continue

                    self.process_gateway_packet(
                        line
                    )

        except (
            ConnectionResetError,
            BrokenPipeError,
            OSError
        ) as error:

            if not rospy.is_shutdown():

                rospy.logwarn(
                    "Gateway receive connection lost: %s",
                    error
                )

        finally:

            with self.connection_lock:

                if self.connection is connection:

                    self.connection = None

            try:
                connection.close()
            except Exception:
                pass

            if not rospy.is_shutdown():

                rospy.logwarn(
                    "ROS 2 gateway receiver stopped"
                )

    # =====================================================
    # IPC packet dispatcher
    # =====================================================

    def process_gateway_packet(self, line):

        try:

            packet = json.loads(
                line.decode("utf-8")
            )

            packet_type = packet.get(
                "type"
            )

            if packet_type == "joint_command":

                self.handle_joint_command(
                    packet
                )

            else:

                rospy.logwarn(
                    "Unknown gateway packet type: %s",
                    packet_type
                )

        except json.JSONDecodeError as error:

            rospy.logerr(
                "Invalid JSON from ROS 2 gateway: %s",
                error
            )

        except Exception as error:

            rospy.logerr(
                "Failed to process gateway packet: %s",
                error
            )

    # =====================================================
    # Validate ROS 2 -> ROS 1 JointCommand
    # =====================================================

    def validate_joint_command(self, data):

        # -------------------------------------------------
        # Required fields
        # -------------------------------------------------

        try:

            mode = int(
                data["mode"]
            )

            names = data["names"]

            position = data["position"]
            velocity = data["velocity"]
            acceleration = data["acceleration"]
            effort = data["effort"]

        except (
            KeyError,
            TypeError,
            ValueError
        ):

            return (
                False,
                "Missing or invalid JointCommand fields"
            )

        # -------------------------------------------------
        # First milestone:
        # POSITION MODE ONLY
        # -------------------------------------------------

        if mode != JointCommand.POSITION_MODE:

            return (
                False,
                "Only POSITION_MODE is currently permitted"
            )

        # -------------------------------------------------
        # Validate names
        # -------------------------------------------------

        if not isinstance(names, list):

            return (
                False,
                "names must be a list"
            )

        if len(names) == 0:

            return (
                False,
                "JointCommand contains no joint names"
            )

        if len(names) > 7:

            return (
                False,
                "JointCommand contains more than 7 joints"
            )

        if len(set(names)) != len(names):

            return (
                False,
                "JointCommand contains duplicate joint names"
            )

        for name in names:

            if name not in VALID_RIGHT_JOINTS:

                return (
                    False,
                    "Invalid Sawyer joint: {}".format(
                        name
                    )
                )

        # -------------------------------------------------
        # Position mode requires one position value
        # per named joint.
        # -------------------------------------------------

        if not isinstance(position, list):

            return (
                False,
                "position must be a list"
            )

        if len(position) != len(names):

            return (
                False,
                "position length does not match names length"
            )

        # -------------------------------------------------
        # Other fields may either:
        #
        #   []
        #
        # or contain one value per named joint.
        # -------------------------------------------------

        for field_name, values in (
            ("velocity", velocity),
            ("acceleration", acceleration),
            ("effort", effort)
        ):

            if not isinstance(values, list):

                return (
                    False,
                    "{} must be a list".format(
                        field_name
                    )
                )

            if len(values) not in (
                0,
                len(names)
            ):

                return (
                    False,
                    "{} length is invalid".format(
                        field_name
                    )
                )

        # -------------------------------------------------
        # Reject NaN / Inf / non-numeric data
        # -------------------------------------------------

        numeric_arrays = (
            position,
            velocity,
            acceleration,
            effort
        )

        for values in numeric_arrays:

            for value in values:

                try:

                    value = float(
                        value
                    )

                except (
                    TypeError,
                    ValueError
                ):

                    return (
                        False,
                        "JointCommand contains "
                        "non-numeric data"
                    )

                if not math.isfinite(
                    value
                ):

                    return (
                        False,
                        "JointCommand contains NaN or Inf"
                    )

        return (
            True,
            ""
        )

    # =====================================================
    # ROS 2 -> ROS 1 JointCommand
    # =====================================================

    def handle_joint_command(self, data):

        # -------------------------------------------------
        # Validate packet first
        # -------------------------------------------------

        valid, reason = \
            self.validate_joint_command(
                data
            )

        if not valid:

            rospy.logwarn(
                "JointCommand rejected: %s",
                reason
            )

            return

        # -------------------------------------------------
        # Reconstruct ROS 1 JointCommand
        # -------------------------------------------------

        msg = JointCommand()

        # -------------------------------------------------
        # Header
        # -------------------------------------------------

        header = data.get(
            "header",
            {}
        )

        sec = int(
            header.get(
                "sec",
                0
            )
        )

        nanosec = int(
            header.get(
                "nanosec",
                0
            )
        )

        msg.header.stamp = rospy.Time.now()

        msg.header.frame_id = \
            header.get(
                "frame_id",
                ""
            )

        # ROS 1 Header contains seq.
        #
        # ROS 2 Header does not contain seq,
        # so there is no ROS 2 value to map.
        msg.header.seq = 0

        # -------------------------------------------------
        # JointCommand fields
        # -------------------------------------------------

        msg.mode = int(
            data["mode"]
        )

        msg.names = list(
            data["names"]
        )

        msg.position = [
            float(value)
            for value in data["position"]
        ]

        msg.velocity = [
            float(value)
            for value in data["velocity"]
        ]

        msg.acceleration = [
            float(value)
            for value in data["acceleration"]
        ]

        msg.effort = [
            float(value)
            for value in data["effort"]
        ]

        # =================================================
        # DRY RUN
        #
        # During the first test we intentionally stop here.
        # =================================================

        if not ENABLE_ROBOT_OUTPUT:

            rospy.loginfo(
                "DRY RUN JointCommand received: "
                "mode=%d names=%s position=%s",
                msg.mode,
                list(msg.names),
                list(msg.position)
            )

            return

        # =================================================
        # Physical robot safety gates
        # =================================================

        if not self.robot_homed:

            rospy.logwarn(
                "JointCommand blocked: "
                "Sawyer is not homed"
            )

            return

        if not self.robot_ready:

            rospy.logwarn(
                "JointCommand blocked: "
                "Sawyer is not ready"
            )

            return

        if not self.robot_enabled:

            rospy.logwarn(
                "JointCommand blocked: "
                "Sawyer is not enabled"
            )

            return

        if self.robot_stopped:

            rospy.logwarn(
                "JointCommand blocked: "
                "Sawyer is stopped"
            )

            return

        if self.robot_error:

            rospy.logwarn(
                "JointCommand blocked: "
                "Sawyer reports an error"
            )

            return

        # =================================================
        # Publish command to Sawyer
        # =================================================

        self.joint_command_pub.publish(
            msg
        )

        rospy.loginfo(
            "JointCommand published to Sawyer: "
            "mode=%d names=%s position=%s",
            msg.mode,
            list(msg.names),
            list(msg.position)
        )

    # =====================================================
    # Sawyer -> ROS 2
    #
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

            "position": list(
                msg.position
            ),

            "velocity": list(
                msg.velocity
            ),

            "effort": list(
                msg.effort
            )
        }

        self.send_packet(
            packet
        )

    # =====================================================
    # Sawyer -> ROS 2
    #
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

        self.send_packet(
            packet
        )

    # =====================================================
    # Sawyer -> ROS 2
    #
    # /robot/state
    # =====================================================

    def robot_state_callback(self, msg):

        # -------------------------------------------------
        # Store robot state for command safety checks
        # -------------------------------------------------

        self.robot_homed = msg.homed
        self.robot_ready = msg.ready
        self.robot_enabled = msg.enabled
        self.robot_stopped = msg.stopped
        self.robot_error = msg.error

        # -------------------------------------------------
        # Forward state to ROS 2
        # -------------------------------------------------

        packet = {

            "type": "robot_state",

            "homed": msg.homed,
            "ready": msg.ready,
            "enabled": msg.enabled,
            "stopped": msg.stopped,
            "error": msg.error,

            # ROS 1 name -> ROS 2 name mapping
            "low_voltage": msg.lowVoltage,

            "estop_button": int(
                msg.estop_button
            ),

            "estop_source": int(
                msg.estop_source
            )
        }

        self.send_packet(
            packet
        )

    # =====================================================
    # Shutdown
    # =====================================================

    def shutdown(self):

        rospy.loginfo(
            "Shutting down Sawyer ROS 1 adapter"
        )

        with self.connection_lock:

            if self.connection is not None:

                try:
                    self.connection.close()
                except Exception:
                    pass

                self.connection = None

        try:
            self.server_socket.close()
        except Exception:
            pass


# =========================================================
# Main
# =========================================================

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
