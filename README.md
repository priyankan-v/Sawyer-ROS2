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
* [ ] Replace or redesign `intera.sh` for dual-environment setup
* [ ] Replace `.rosinstall` / `wstool` workflow with `.repos` / `vcs`

## Phase 2 - Low-Risk Software Ports

* [ ] Port utility modules
* [ ] Port read-only robot interfaces
* [ ] Add QoS, executor, timing, shutdown, and parameter design

## Phase 3 - Device and Control Interfaces

* [ ] Port I/O interface
* [ ] Port gripper interface
* [ ] Port head interface
* [ ] Port navigator interface
* [ ] Port camera interface
* [ ] Port safety and robot-enable functionality
* [ ] Port IK/FK services
* [ ] Port `limb.py` in stages
* [ ] Introduce position control
* [ ] Port the motion interface

## Phase 4 - Actions, Parameters, and Execution

* [ ] Port ROS 1 `actionlib` functionality to ROS 2 actions
* [ ] Replace dynamic reconfigure with ROS 2 parameters
* [ ] Port the joint trajectory action server
* [ ] Port every executable under `intera_interface/scripts`
* [ ] Port ROS 1 launch files

## Phase 5 - Examples and Validation

* [ ] Port `intera_examples` in increasing order of hardware risk
* [ ] Run staged simulation tests
* [ ] Run mock-interface tests
* [ ] Run read-only hardware tests
* [ ] Run controlled hardware motion tests

---

# Immediate Next Work Items

1. Port ROS 1 launch files for remaining packages and create ROS 2 launch replacements.

2. Port utility modules and read-only robot interfaces in `intera_interface`.

3. Establish a `ros1_bridge` or custom gateway path for communication with the physical Sawyer robot.
