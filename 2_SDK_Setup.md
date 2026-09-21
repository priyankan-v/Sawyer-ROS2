# Install the Sawyer Intera SDK (ROS 1 Noetic)

Complete [Docker setup](1_Docker_Setup.md) first. The commands below assume its
`sawyer_noetic_pr` container is running, this repository is mounted at
`/root/sawyer_ros2`, and the container can reach the Sawyer controller. Run all
commands **inside the container** unless a step says otherwise. This builds the
ROS 1 SDK workspace; it does not install the SDK into a ROS 2 workspace.

## 1. Enter the container

On the Ubuntu host:

```bash
docker start sawyer_noetic_pr  # Only needed if the container is stopped
docker exec -it sawyer_noetic_pr bash
```

## 2. Link the SDK packages into a catkin workspace

The three SDK repositories already live at the root of this checkout. Link
their catkin packages into `ros1_ws/src` so there is only one copy of each
package. From inside the container:

```bash
cd /root/sawyer_ros2
mkdir -p ros1_ws/src

for package in intera_common/* intera_sdk/* sawyer_robot/*; do
    [ -f "$package/package.xml" ] || continue
    link="ros1_ws/src/${package##*/}"
    if [ ! -e "$link" ] && [ ! -L "$link" ]; then
        ln -s "../../$package" "$link"
    fi
done

ls -l ros1_ws/src
```

The result should include `intera_core_msgs`, `intera_motion_msgs`,
`intera_interface`, `intera_examples`, `sawyer_description`, and the three
metapackages. If a name already exists in `src`, check that it points to the
matching package in this checkout before building.

## 3. Install dependencies

```bash
source /opt/ros/noetic/setup.bash
apt update
apt install -y \
    python3-rosdep \
    build-essential \
    ros-noetic-joystick-drivers \
    ros-noetic-rospy-message-converter

if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    rosdep init
fi
rosdep update

cd /root/sawyer_ros2/ros1_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic \
    --skip-keys "joystick_drivers rospy_message_converter" -r -y
```

The packages declare `joystick_drivers` and `rospy_message_converter`, but
rosdep has no Noetic rules for these keys in this setup. The explicit
`apt install` supplies both; `--skip-keys` only skips rosdep's lookup for them.
Rosdep installs the remaining declared dependencies.

## 4. Build and check the workspace

```bash
cd /root/sawyer_ros2/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make --force-cmake
source devel/setup.bash

rospack find intera_core_msgs
rospack find intera_motion_msgs
rospack find intera_interface
rospack find intera_examples
rospack find sawyer_description
rosmsg show intera_core_msgs/JointCommand
```

The `rospack` commands should print paths under `ros1_ws/src`; `rosmsg`
should print the `JointCommand` fields. These checks do not require a live
robot.

## 5. Configure the robot connection

###
```bash
cd /root/sawyer_ros2/ros1_ws/src
catkin_create_pkg sawyer_ros1_adapter rospy sensor_msgs
mkdir -p sawyer_ros1_adapter/scripts

nano sawyer_ros1_adapter/scripts/joint_state_adapter.py
```

Paste the following in the above python file
```python
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
```
```bash
chmod +x sawyer_ros1_adapter/scripts/joint_state_adapter.py

cd /root/sawyer_ros2/ros1_ws

source /opt/ros/noetic/setup.bash
catkin_make

source devel/setup.bash

rospack find sawyer_ros1_adapter
```

Wait and come to the ROS2 workspace in another terminal
###
```bash
cd ~/sawyer_ros2

mkdir -p ros2_ws/src

cd ros2_ws/src

source /opt/ros/humble/setup.bash

ros2 pkg create \
    --build-type ament_python \
    --dependencies rclpy sensor_msgs \
    --node-name joint_state_gateway \
    sawyer_ros2_gateway
```

Update the python file '~/sawyer_ros2/ros2_ws/src/sawyer_ros2_gateway/sawyer_ros2_gateway/joint_state_gateway.py'
```
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
```

###
```bash
cd ~/sawyer_ros2/ros2_ws

source /opt/ros/humble/setup.bash

colcon build

cd ~/sawyer_ros2/ros2_ws

source /opt/ros/humble/setup.bash

colcon build
```

###
```bash

```

