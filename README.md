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
* [x] Port utility modules (ROS2-safe package exports, wait/dataflow, and script-level utility ports)
* [x] Port read-only robot interfaces (RobotParams/JointLimits/head/navigator/camera baselines)
* [x] Start ROS 2 read-only ports: `RobotParams`, `JointLimits`, `wait_for` utility
* [ ] Add QoS, executor, timing, shutdown, and parameter design

## Phase 3 - Device and Control Interfaces

* [x] Add ROS 2 baseline I/O interface modules (`io_interface_ros2.py`, `io_command_ros2.py`)
* [x] Add ROS 2 baseline gripper interface (`gripper_ros2.py`)
* [x] Add ROS 2 baseline head interface (`head_ros2.py`) and example (`head_wobbler_ros2.py`)
* [x] Add ROS 2 baseline navigator interface (`navigator_ros2.py`) and example (`navigator_io_ros2.py`)
* [x] Add ROS 2 baseline camera interface (`camera_ros2.py`) and example (`camera_display_ros2.py`)
* [x] Add ROS 2 baseline for safety and robot-enable functionality (`robot_enable_ros2.py`)
* [x] Add ROS 2 baseline cuff interface (`cuff_ros2.py`)
* [x] Port IK/FK service clients via `LimbROS2` wrappers (`ik_request`, `fk_request`)
* [x] Add ROS 2 baseline limb runtime layer (`limb_ros2.py`) with command/state and IK/FK client wrappers
* [x] Introduce position control baseline (`LimbROS2.set_joint_positions`, joystick and trajectory flows)
* [ ] Port the motion interface

## Phase 4 - Actions, Parameters, and Execution

* [x] Port ROS 1 `actionlib` functionality to ROS 2 actions for launch-critical trajectory path
* [ ] Replace dynamic reconfigure with ROS 2 parameters
* [x] Port the joint trajectory action server for ROS 2 launch/runtime baseline
* [x] Port every executable under `intera_interface/scripts` (ROS2 counterparts added for enable/home/calibrate/io-config/urdf)
* [x] Port ROS 1 launch files used in launch-critical paths
* [x] Add initial ROS 2 launch.py equivalents for `intera_examples` and `intera_interface`

## Phase 5 - Examples and Validation

* [x] Port additional medium-risk ROS2 gripper examples: `gripper_keyboard_ros2.py`, `gripper_cuff_control_ros2.py`
* [x] Port additional ROS2 examples: `lights_blink_ros2.py`, `joint_trajectory_file_playback_ros2.py`
* [ ] Run staged simulation tests
* [x] Run mock-interface tests (host smoke validation for ROS2 scripts with hardware-gated failure classification)
* [ ] Run read-only hardware tests
* [ ] Run controlled hardware motion tests

---

# Immediate Next Work Items

1. Complete bridge host prerequisites and execute `tools/bridge/ros1_ros2_bridge_smoke.sh` end-to-end with live ROS1+ROS2 processes.

2. Run hardware validation for ROS2 interfaces/examples and close parity gaps (`head/navigator/camera/gripper/cuff/lights`, plus trajectory playback).

3. Port or redesign the remaining motion-interface stack (`intera_motion_interface`) for ROS2-native runtime.

4. Complete QoS/executor/parameter hardening pass for runtime nodes beyond baseline defaults.