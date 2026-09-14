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

* [ ] Port `intera_core_msgs`
* [ ] Port `intera_motion_msgs`
* [ ] Port `intera_tools_description`
* [ ] Port `sawyer_description` package metadata and build files
* [ ] Validate `colcon build` for the first package batch
* [ ] Validate Sawyer visualization path in RViz2 launch stack

## Phase 1 - ROS 1 / ROS 2 Coexistence

* [ ] Establish ROS 1-to-ROS 2 real-robot communication
* [ ] Create ROS 1-to-ROS 2 bridge bring-up plan and smoke-test script
* [ ] Install ROS 1 + `ros1_bridge` prerequisites on bridge host and run live telemetry smoke test
* [ ] Replace or redesign `intera.sh` for dual-environment setup
* [ ] Replace `.rosinstall` / `wstool` workflow with `.repos` / `vcs`

## Phase 2 - Low-Risk Software Ports

* [ ] Port build metadata (`package.xml`, `CMakeLists.txt`, `setup.py`) for `intera_interface` and `intera_examples` to ROS 2 ament
* [ ] Port utility modules (ROS2-safe package exports, wait/dataflow, and script-level utility ports)
* [ ] Port read-only robot interfaces (RobotParams/JointLimits/head/navigator/camera baselines)
* [ ] Start ROS 2 read-only ports: `RobotParams`, `JointLimits`, `wait_for` utility
* [ ] Add QoS, executor, timing, shutdown, and parameter design

## Phase 3 - Device and Control Interfaces

* [ ] Add ROS 2 baseline I/O interface modules (`io_interface_ros2.py`, `io_command_ros2.py`)
* [ ] Add ROS 2 baseline gripper interface (`gripper_ros2.py`)
* [ ] Add ROS 2 baseline head interface (`head_ros2.py`) and example (`head_wobbler_ros2.py`)
* [ ] Add ROS 2 baseline navigator interface (`navigator_ros2.py`) and example (`navigator_io_ros2.py`)
* [ ] Add ROS 2 baseline camera interface (`camera_ros2.py`) and example (`camera_display_ros2.py`)
* [ ] Add ROS 2 baseline for safety and robot-enable functionality (`robot_enable_ros2.py`)
* [ ] Add ROS 2 baseline cuff interface (`cuff_ros2.py`)
* [ ] Port IK/FK service clients via `LimbROS2` wrappers (`ik_request`, `fk_request`)
* [ ] Add ROS 2 baseline limb runtime layer (`limb_ros2.py`) with command/state and IK/FK client wrappers
* [ ] Introduce position control baseline (`LimbROS2.set_joint_positions`, joystick and trajectory flows)
* [ ] Port the motion interface

## Phase 4 - Actions, Parameters, and Execution

* [ ] Port ROS 1 `actionlib` functionality to ROS 2 actions for launch-critical trajectory path
* [ ] Replace dynamic reconfigure with ROS 2 parameters
* [ ] Port the joint trajectory action server for ROS 2 launch/runtime baseline
* [ ] Port every executable under `intera_interface/scripts` (ROS2 counterparts added for enable/home/calibrate/io-config/urdf)
* [ ] Port ROS 1 launch files used in launch-critical paths
* [ ] Add initial ROS 2 launch.py equivalents for `intera_examples` and `intera_interface`

## Phase 5 - Examples and Validation

* [ ] Port additional medium-risk ROS2 gripper examples: `gripper_keyboard_ros2.py`, `gripper_cuff_control_ros2.py`
* [ ] Port additional ROS2 examples: `lights_blink_ros2.py`, `joint_trajectory_file_playback_ros2.py`
* [ ] Run staged simulation tests
* [ ] Run mock-interface tests (host smoke validation for ROS2 scripts with hardware-gated failure classification)
* [ ] Run read-only hardware tests
* [ ] Run controlled hardware motion tests

---