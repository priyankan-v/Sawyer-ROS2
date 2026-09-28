## Change the code in ros1
```bash
cd /root/sawyer_ros2/ros1_ws/src

rm sawyer_ros1_adapter/scripts/joint_state_adapter.py

nano sawyer_ros1_adapter/scripts/joint_state_adapter.py
```
Select all: Alt + A
Delete: Alt + T
Save : Ctrl + O
Exit: Ctrl + X

```python
#!/usr/bin/env python3

import rospy
import socket
import json
import time

from sensor_msgs.msg import JointState


UDP_IP = "127.0.0.1"
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

sample_count = 0
adapter_latency_sum = 0.0


def joint_state_callback(msg):
    global sample_count
    global adapter_latency_sum

    # t1: ROS1 callback receives Sawyer message
    t1 = time.monotonic_ns()

    packet = {
        "name": list(msg.name),
        "position": list(msg.position),
        "velocity": list(msg.velocity),
        "effort": list(msg.effort)
    }

    # t2: just before sending to ROS2 gateway
    t2 = time.monotonic_ns()

    packet["t1"] = t1
    packet["t2"] = t2

    data = json.dumps(packet).encode("utf-8")

    sock.sendto(data, (UDP_IP, UDP_PORT))

    adapter_latency_ms = (t2 - t1) / 1e6

    sample_count += 1
    adapter_latency_sum += adapter_latency_ms

    if sample_count % 100 == 0:
        average = adapter_latency_sum / sample_count

        rospy.loginfo(
            "Adapter samples: %d | Current: %.4f ms | Average: %.4f ms",
            sample_count,
            adapter_latency_ms,
            average
        )


def main():

    rospy.init_node("sawyer_ros1_adapter")

    rospy.Subscriber(
        "/robot/joint_states",
        JointState,
        joint_state_callback,
        queue_size=1
    )

    rospy.loginfo("Sawyer ROS1 adapter started")
    rospy.loginfo("Forwarding /robot/joint_states to UDP port %d", UDP_PORT)

    rospy.spin()


if __name__ == "__main__":
    main()
```

```bash
chmod +x ~/sawyer_ros2/ros1_ws/src/sawyer_ros1_adapter/scripts/joint_state_adapter.py

source /opt/ros/noetic/setup.bash
source ~/sawyer_ros2/ros1_ws/devel/setup.bash

export ROS_MASTER_URI=http://169.254.121.3:11311
export ROS_IP=169.254.121.2
unset ROS_HOSTNAME

rosrun sawyer_ros1_adapter joint_state_adapter.py
```

## Change gateway code in ros2_ws
'ros2_ws/src/sawyer_ros2_gateway/sawyer_ros2_gateway/joint_state_gateway.py'


```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import socket
import json
import time
import threading

import numpy as np


UDP_IP = "127.0.0.1"
UDP_PORT = 5005


class JointStateGateway(Node):

    def __init__(self):

        super().__init__("joint_state_gateway")

        self.publisher = self.create_publisher(
            JointState,
            "/joint_states",
            10
        )

        self.adapter_latencies = []
        self.ipc_latencies = []
        self.gateway_latencies = []
        self.total_latencies = []

        self.sock = socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        )

        self.sock.bind((UDP_IP, UDP_PORT))

        self.thread = threading.Thread(
            target=self.receive_loop,
            daemon=True
        )

        self.thread.start()

        self.get_logger().info(
            "ROS2 joint-state gateway started"
        )


    def receive_loop(self):

        while rclpy.ok():

            data, _ = self.sock.recvfrom(65535)

            # t3: packet reaches ROS2 gateway
            t3 = time.monotonic_ns()

            packet = json.loads(
                data.decode("utf-8")
            )

            t1 = packet["t1"]
            t2 = packet["t2"]

            msg = JointState()

            msg.header.stamp = (
                self.get_clock().now().to_msg()
            )

            msg.name = packet["name"]
            msg.position = packet["position"]
            msg.velocity = packet["velocity"]
            msg.effort = packet["effort"]

            self.publisher.publish(msg)

            # t4: publish call completed
            t4 = time.monotonic_ns()

            adapter_ms = (t2 - t1) / 1e6
            ipc_ms = (t3 - t2) / 1e6
            gateway_ms = (t4 - t3) / 1e6
            total_ms = (t4 - t1) / 1e6

            self.adapter_latencies.append(adapter_ms)
            self.ipc_latencies.append(ipc_ms)
            self.gateway_latencies.append(gateway_ms)
            self.total_latencies.append(total_ms)

            if len(self.total_latencies) % 100 == 0:
                self.print_statistics()


    def print_statistics(self):

        adapter = np.array(self.adapter_latencies)
        ipc = np.array(self.ipc_latencies)
        gateway = np.array(self.gateway_latencies)
        total = np.array(self.total_latencies)

        self.get_logger().info(
            "\n"
            "================ LATENCY RESULTS ================\n"
            f"Samples          : {len(total)}\n"
            f"Adapter mean     : {np.mean(adapter):.4f} ms\n"
            f"IPC mean         : {np.mean(ipc):.4f} ms\n"
            f"Gateway mean     : {np.mean(gateway):.4f} ms\n"
            "\n"
            f"Total mean       : {np.mean(total):.4f} ms\n"
            f"Total median     : {np.median(total):.4f} ms\n"
            f"Total minimum    : {np.min(total):.4f} ms\n"
            f"Total maximum    : {np.max(total):.4f} ms\n"
            f"Total std dev    : {np.std(total):.4f} ms\n"
            f"Total P95        : {np.percentile(total, 95):.4f} ms\n"
            f"Total P99        : {np.percentile(total, 99):.4f} ms\n"
            "================================================="
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

```bash
cd ~/sawyer_ros2/ros2_ws

colcon build --packages-select sawyer_ros2_gateway

source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run sawyer_ros2_gateway joint_state_gateway
```

```bash

```

# ROS1 Adapter to ROS2 Gateway Latency Measurement

## Overview

The latency measurement is based on recording timestamps at four important points in the communication path:

```text
Sawyer ROS1 Topic
      |
      v
ROS1 Adapter
      |
      | t1 = message received by ROS1 callback
      |
      | message conversion / preparation
      |
      | t2 = just before sending through IPC
      v
Local IPC (UDP)
      |
      | t3 = packet received by ROS2 gateway
      v
ROS2 Gateway
      |
      | message reconstruction
      |
      | ROS2 publish()
      |
      | t4 = immediately after publish() returns
      v
ROS2 /joint_states
```

The four timestamps are measured using:

```python
time.monotonic_ns()
```

This returns a high-resolution monotonic clock value in nanoseconds.

A monotonic clock is used because it only moves forward and is not affected by normal system clock corrections.

---

## Timestamp t1: ROS1 Adapter Receives the Message

The first timestamp is taken immediately when the ROS1 subscriber callback starts.

```python
def joint_state_callback(msg):

    t1 = time.monotonic_ns()
```

At this point, the Sawyer ROS1 message has already arrived at the adapter.

Therefore:

```text
t1 = ROS1 adapter receive time
```

This timestamp is the starting point of the measured ROS1-to-ROS2 migration latency.

---

## Timestamp t2: Adapter Sends the IPC Packet

After the ROS1 message is converted into the packet that will be sent to the ROS2 gateway, another timestamp is recorded.

```python
packet = {
    "name": list(msg.name),
    "position": list(msg.position),
    "velocity": list(msg.velocity),
    "effort": list(msg.effort)
}

t2 = time.monotonic_ns()
```

Then `t1` and `t2` are included in the packet:

```python
packet["t1"] = t1
packet["t2"] = t2
```

The packet is then serialized and sent:

```python
data = json.dumps(packet).encode("utf-8")

sock.sendto(
    data,
    (UDP_IP, UDP_PORT)
)
```

The adapter processing latency is calculated as:

```text
Adapter latency = t2 - t1
```

or:

```python
adapter_latency_ms = (t2 - t1) / 1e6
```

Since the timestamps are in nanoseconds, dividing by `1e6` converts the result to milliseconds.

Therefore:

```text
L_adapter = t2 - t1
```

This represents approximately:

```text
ROS1 callback
      ↓
Extract joint-state data
      ↓
Construct IPC packet
      ↓
Ready to send
```

---

## Timestamp t3: ROS2 Gateway Receives the IPC Packet

The ROS2 gateway waits for packets from the ROS1 adapter:

```python
data, _ = self.sock.recvfrom(65535)
```

Immediately after the packet is received, another timestamp is recorded:

```python
t3 = time.monotonic_ns()
```

Therefore:

```text
t3 = IPC packet arrival time at ROS2 gateway
```

The IPC latency is then:

```text
IPC latency = t3 - t2
```

or:

```python
ipc_ms = (t3 - t2) / 1e6
```

Therefore:

```text
L_IPC = t3 - t2
```

This includes the time required for the packet to move from the ROS1 adapter process to the ROS2 gateway process.

In this implementation, the communication is through local UDP:

```text
127.0.0.1:5005
```

so the packet remains on the same physical computer.

---

## Timestamp t4: ROS2 Message Is Published

After receiving the UDP packet, the ROS2 gateway reconstructs a ROS2 `JointState` message:

```python
msg = JointState()

msg.name = packet["name"]
msg.position = packet["position"]
msg.velocity = packet["velocity"]
msg.effort = packet["effort"]
```

The message is then published:

```python
self.publisher.publish(msg)
```

Immediately after `publish()` returns, another timestamp is taken:

```python
t4 = time.monotonic_ns()
```

Therefore:

```text
t4 = ROS2 publish call completed
```

The ROS2 gateway processing latency is:

```text
Gateway latency = t4 - t3
```

or:

```python
gateway_ms = (t4 - t3) / 1e6
```

Therefore:

```text
L_gateway = t4 - t3
```

This represents approximately:

```text
UDP packet received
      ↓
JSON decoding
      ↓
ROS2 message construction
      ↓
ROS2 publish()
```

---

# Total Latency

The total measured latency is:

```text
Total latency = t4 - t1
```

or:

```python
total_ms = (t4 - t1) / 1e6
```

Therefore:

```text
L_total = t4 - t1
```

Since:

```text
L_adapter = t2 - t1
L_IPC     = t3 - t2
L_gateway = t4 - t3
```

the total latency is approximately:

```text
L_total =
    L_adapter
    + L_IPC
    + L_gateway
```

or:

```text
t4 - t1
=
(t2 - t1)
+
(t3 - t2)
+
(t4 - t3)
```

---

# Complete Timing Diagram

```text
                    ROS1 SIDE
                       |
Sawyer                 |
/robot/joint_states    |
        |              |
        v              |
 ROS1 callback         |
        |
        | t1
        |
        |<------ Adapter processing ------>|
        |
        | t2
        |
        +--------- UDP / IPC ------------+
                                         |
                                         | t3
                                         v
                                ROS2 Gateway
                                         |
                                         | message conversion
                                         |
                                         | publish()
                                         |
                                         | t4
                                         v
                                ROS2 /joint_states
```

The measured quantities are:

```text
Adapter latency:
    t2 - t1

IPC latency:
    t3 - t2

Gateway latency:
    t4 - t3

Total migration latency:
    t4 - t1
```

---

# Example

Suppose the timestamps correspond to:

```text
t1 = 0.000 ms
t2 = 0.050 ms
t3 = 0.220 ms
t4 = 0.300 ms
```

Then:

```text
Adapter latency
= 0.050 - 0.000
= 0.050 ms
```

```text
IPC latency
= 0.220 - 0.050
= 0.170 ms
```

```text
Gateway latency
= 0.300 - 0.220
= 0.080 ms
```

and:

```text
Total latency
= 0.300 - 0.000
= 0.300 ms
```

The same result can be obtained by adding the three components:

```text
0.050 + 0.170 + 0.080
= 0.300 ms
```

---

# Why `time.monotonic_ns()` Is Used

The code uses:

```python
time.monotonic_ns()
```

instead of:

```python
time.time()
```

because latency measurement requires measuring elapsed time accurately.

`time.monotonic_ns()` has two useful properties:

1. It provides nanosecond-resolution values.
2. The clock cannot move backward because of system clock synchronization or manual clock changes.

This makes it suitable for short-duration performance measurements.

---

# Statistical Analysis

One latency value is not enough to characterize system performance.

The gateway therefore stores many samples:

```python
self.total_latencies.append(total_ms)
```

After many messages, statistics are calculated.

For example:

```python
np.mean(total)
```

gives the average latency.

```python
np.median(total)
```

gives the median latency.

```python
np.max(total)
```

gives the worst observed latency.

```python
np.percentile(total, 95)
```

gives the 95th percentile latency.

```python
np.percentile(total, 99)
```

gives the 99th percentile latency.

For example:

```text
Total mean    : 0.31 ms
Total median  : 0.28 ms
Total max     : 2.10 ms
Total P95     : 0.49 ms
Total P99     : 0.82 ms
```

A P95 latency of:

```text
0.49 ms
```

means that approximately 95% of the measured messages had a latency less than or equal to:

```text
0.49 ms
```

Similarly, a P99 of:

```text
0.82 ms
```

means approximately 99% of the samples were below or equal to that value.

These percentile values are useful because occasional operating-system scheduling delays can cause latency spikes.

---

# What This Measurement Includes

The total latency:

```text
t4 - t1
```

measures the additional delay introduced by the migration layer:

```text
ROS1 callback
      ↓
ROS1 adapter processing
      ↓
Serialization
      ↓
Local IPC
      ↓
Deserialization
      ↓
ROS2 message construction
      ↓
ROS2 publish call
```

This is useful for evaluating whether the ROS1-to-ROS2 adapter and gateway architecture introduces significant additional delay.

---

# What This Measurement Does NOT Include

This experiment does not measure the complete latency from the Sawyer robot hardware to a final ROS2 subscriber.

In particular, it does not include the time between:

```text
Sawyer sensor/controller
      ↓
Sawyer creates ROS1 message
      ↓
Network transmission
      ↓
ROS1 adapter receives message
```

because the measurement begins at:

```text
t1
```

when the ROS1 adapter callback receives the message.

It also does not measure the time required for a separate ROS2 subscriber to receive the ROS2 message.

The final timestamp:

```text
t4
```

is taken immediately after:

```python
self.publisher.publish(msg)
```

returns.

Therefore, the current measurement should be described as:

```text
ROS1 adapter-to-ROS2 publish latency
```

or:

```text
ROS1-to-ROS2 migration-layer latency
```

rather than true end-to-end Sawyer-to-ROS2-subscriber latency.

---

# Frequency vs Latency

Frequency and latency are different measurements.

The ROS1 input frequency can be measured using:

```bash
rostopic hz /robot/joint_states
```

The ROS2 output frequency can be measured using:

```bash
ros2 topic hz /joint_states
```

These measurements tell us whether the gateway preserves the incoming message rate.

For example:

```text
ROS1 input frequency : 100.1 Hz
ROS2 output frequency: 99.9 Hz
```

suggests that almost all messages are being forwarded.

Latency measurements answer a different question:

```text
How much additional delay does the adapter/gateway introduce?
```

Therefore, a useful performance evaluation should include both:

```text
1. ROS1 input frequency
2. ROS2 output frequency
3. Mean latency
4. Median latency
5. P95 latency
6. P99 latency
7. Maximum observed latency
```

Together, these measurements show both the throughput and timing behavior of the ROS1-to-ROS2 migration architecture.

One wording I would use with your PI is **“migration-layer latency”**, because the current `t1 → t4` measurement does not yet include Sawyer-to-adapter network delay or ROS2 subscriber reception delay.