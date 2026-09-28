## On the host

```bash
cd ~/sawyer_ros2/ros2_ws/src

source /opt/ros/humble/setup.bash

ros2 pkg create \
    --build-type ament_cmake \
    intera_core_msgs
```

```bash
cd intera_core_msgs
mkdir -p msg
```

```bash
cp ~/sawyer_ros2/intera_common/intera_core_msgs/msg/EndpointState.msg msg/

cp ~/sawyer_ros2/intera_common/intera_core_msgs/msg/RobotAssemblyState.msg msg/

cp ~/sawyer_ros2/intera_common/intera_core_msgs/msg/JointCommand.msg msg/
```
## Change the following files

`RobotAssemblyState.msg`
```text
bool homed
bool ready
bool enabled
bool stopped
bool error
bool low_voltage

uint8 estop_button

uint8 ESTOP_BUTTON_UNPRESSED=0
uint8 ESTOP_BUTTON_PRESSED=1
uint8 ESTOP_BUTTON_UNKNOWN=2
uint8 ESTOP_BUTTON_RELEASED=3

uint8 estop_source

uint8 ESTOP_SOURCE_NONE=0
uint8 ESTOP_SOURCE_USER=1
uint8 ESTOP_SOURCE_UNKNOWN=2
uint8 ESTOP_SOURCE_FAULT=3
uint8 ESTOP_SOURCE_ENGINE=4
```

`EndpointState.msg`
```text
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Twist twist
geometry_msgs/Wrench wrench
bool valid
```

`JointCommand.msg`
```text
std_msgs/Header header

int32 mode

string[] names

float64[] position
float64[] velocity
float64[] acceleration
float64[] effort

int32 POSITION_MODE=1
int32 VELOCITY_MODE=2
int32 TORQUE_MODE=3
int32 TRAJECTORY_MODE=4
```

`CMakeLists.txt`
```text
cmake_minimum_required(VERSION 3.8)

project(intera_core_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/EndpointState.msg"
  "msg/RobotAssemblyState.msg"
  "msg/JointCommand.msg"

  DEPENDENCIES
    std_msgs
    geometry_msgs
)

ament_export_dependencies(
  rosidl_default_runtime
)

ament_package()
```

```bash
cd ~/sawyer_ros2/ros2_ws

rm -rf build/intera_core_msgs
rm -rf install/intera_core_msgs
```

```bash
source /opt/ros/humble/setup.bash

colcon build --packages-select intera_core_msgs
```

### Verificaton
```bash
source ~/sawyer_ros2/ros2_ws/install/setup.bash

ros2 interface show intera_core_msgs/msg/RobotAssemblyState
ros2 interface show intera_core_msgs/msg/EndpointState
ros2 interface show intera_core_msgs/msg/JointCommand
```

# Replace ROS1 adapter

```bash
nano /root/sawyer_ros2/ros1_ws/src/sawyer_ros1_adapter/scripts/joint_state_adapter.py
```

```python
#!/usr/bin/env python3

import json
import socket
import threading

import rospy

from sensor_msgs.msg import JointState

from intera_core_msgs.msg import (EndpointState, RobotAssemblyState, )

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
```

```bash
chmod +x \
/root/sawyer_ros2/ros1_ws/src/sawyer_ros1_adapter/scripts/joint_state_adapter.py
```

```bash
cd /root/sawyer_ros2/ros1_ws

source /opt/ros/noetic/setup.bash

catkin_make

source devel/setup.bash

rosrun sawyer_ros1_adapter joint_state_adapter.py
```
# Replace ROS2 Gateway
`nano ~/sawyer_ros2/ros2_ws/src/sawyer_ros2_gateway/sawyer_ros2_gateway/joint_state_gateway.py`

```python
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
```

```bash
cd ~/sawyer_ros2/ros2_ws

source /opt/ros/humble/setup.bash

colcon build \
    --packages-select \
    intera_core_msgs \
    sawyer_ros2_gateway

source install/setup.bash

ros2 run sawyer_ros2_gateway joint_state_gateway
```

## In another terminal
```bash
ros2 topic echo /joint_states --once

ros2 topic echo /robot/limb/right/endpoint_state --once

ros2 topic echo /robot/state --once
```

```bash

```
