# Sawyer ROS 2 Migration Notes

## ROS 1 Dependencies Currently Used by the SDK

```bash
git-core
python-argparse
python-wstool
python-vcstools
python-rosdep
ros-melodic-control-msgs
ros-melodic-joystick-drivers
ros-melodic-xacro
ros-melodic-tf2-ros
ros-melodic-rviz
ros-melodic-cv-bridge
ros-melodic-actionlib
ros-melodic-actionlib-msgs
ros-melodic-dynamic-reconfigure
ros-melodic-trajectory-msgs
ros-melodic-rospy-message-converter
```

## Current ROS 1 SDK Bootstrap

```bash
./intera.sh
```

## Critical Architecture Constraint

`intera.sh` sets `ROS_MASTER_URI` toward Sawyer, so the workstation SDK communicates with the robot's ROS 1 master.

Converting `rospy` to `rclpy` alone does **not** make the physical Sawyer robot ROS 2-native.

A ROS 1-to-ROS 2 bridge or gateway stage is required before commanding the hardware from ROS 2 nodes.

---

# Migration Execution Order

## Phase 0 - Interface and Build Foundation

* [x] Port `intera_core_msgs`
* [x] Port `intera_motion_msgs`
* [x] Port `intera_tools_description`
* [x] Port `sawyer_description` package metadata and build files
* [x] Validate `colcon build` for the first package batch
* [x] Validate Sawyer visualization path in RViz2 launch stack

## Phase 1 - ROS 1 / ROS 2 Coexistence

* [ ] Establish ROS 1-to-ROS 2 real-robot communication
* [x] Create ROS 1-to-ROS 2 bridge bring-up plan and smoke-test script
* [ ] Install ROS 1 + `ros1_bridge` prerequisites on bridge host and run live telemetry smoke test
* [ ] Replace or redesign `intera.sh` for dual-environment setup
* [ ] Replace `.rosinstall` / `wstool` workflow with `.repos` / `vcs`

## Phase 2 - Low-Risk Software Ports

* [x] Port build metadata (`package.xml`, `CMakeLists.txt`, `setup.py`) for `intera_interface` and `intera_examples` to ROS 2 ament
* [ ] Port utility modules
* [ ] Port read-only robot interfaces
* [x] Start ROS 2 read-only ports: `RobotParams`, `JointLimits`, `wait_for` utility
* [ ] Add QoS, executor, timing, shutdown, and parameter design

## Phase 3 - Device and Control Interfaces

* [ ] Port I/O interface
* [ ] Port gripper interface
* [ ] Port head interface
* [ ] Port navigator interface
* [ ] Port camera interface
* [x] Add ROS 2 baseline for safety and robot-enable functionality (`robot_enable_ros2.py`)
* [ ] Port IK/FK services
* [x] Add ROS 2 baseline limb runtime layer (`limb_ros2.py`) with command/state and IK/FK client wrappers
* [ ] Introduce position control
* [ ] Port the motion interface

## Phase 4 - Actions, Parameters, and Execution

* [x] Port ROS 1 `actionlib` functionality to ROS 2 actions for launch-critical trajectory path
* [ ] Replace dynamic reconfigure with ROS 2 parameters
* [x] Port the joint trajectory action server for ROS 2 launch/runtime baseline
* [ ] Port every executable under `intera_interface/scripts`
* [ ] Port ROS 1 launch files
* [x] Add initial ROS 2 launch.py equivalents for `intera_examples` and `intera_interface`

## Phase 5 - Examples and Validation

* [ ] Port `intera_examples` in increasing order of hardware risk
* [ ] Run staged simulation tests
* [ ] Run mock-interface tests
* [ ] Run read-only hardware tests
* [ ] Run controlled hardware motion tests

---

# Immediate Next Work Items

1. Integrate `LimbROS2` and `RobotEnableROS2` into selected launch-critical scripts where safe.

2. Extend read-only ROS 2 interface coverage (`head`, `navigator`, `camera` state paths).

3. Install ROS 1 and `ros1_bridge` on the bridge host and execute `tools/bridge/ros1_ros2_bridge_smoke.sh` end-to-end.
