# ROS 2 Migration Tracker

This tracker converts the high-level plan into concrete implementation gates.

## Completed in this session

- Converted intera_core_msgs to ROS 2 interface package metadata and build rules.
- Converted intera_motion_msgs to ROS 2 interface package metadata and build rules.
- Removed ROS 1 actionlib build/runtime dependencies from both interface packages.
- Converted intera_tools_description to ROS 2 ament package metadata/build rules.
- Converted sawyer_description to ROS 2 ament package metadata/build rules.
- Built and validated all four migrated packages with colcon on ROS 2 Humble.
- Added a ROS 2 Sawyer visualization launch entrypoint and RViz2 config.
- Validated xacro generation and robot_state_publisher startup from the ROS 2 launch path.
- Ported ROS 1 XML launch files in intera_examples to ROS 2 launch.py equivalents.
- Added shared ROS 2 trajectory action server launch entrypoint for intera_interface.
- Started read-only intera_interface ROS 2 modules: robot params, joint limits, and wait utility.
- Added a ROS 1/ROS 2 bridge smoke-test script and executed prerequisite checks.
- Ported intera_interface and intera_examples build metadata to ROS 2 ament_cmake_python.
- Built intera_interface and intera_examples successfully with colcon as ROS 2 packages.
- Verified ROS 2 launch discovery for intera_interface and intera_examples launch files.
- Added ROS 2-native runtime executables for launch-critical flows:
  - intera_interface/scripts/joint_trajectory_action_server_ros2.py
  - intera_examples/scripts/joint_trajectory_client_ros2.py
  - intera_examples/scripts/joint_position_joystick_ros2.py
  - intera_examples/scripts/gripper_joystick_ros2.py
  - intera_examples/src/intera_external_devices/joystick_ros2.py
- Updated ROS 2 launch files to execute ROS 2-native runtime scripts.
- Rebuilt intera_interface and intera_examples and revalidated `--show-args` launch discovery.
- Hardened ROS 2 trajectory action server behavior with:
  - goal validation for point dimensions and monotonic time
  - path tolerance violation aborts (`PATH_TOLERANCE_VIOLATED`)
  - goal tolerance + stopped velocity tolerance checks (`GOAL_TOLERANCE_VIOLATED`)
  - launch argument `stopped_velocity_tolerance` for runtime tuning
- Added next runtime-layer ROS 2 modules:
  - intera_interface/src/intera_interface/robot_enable_ros2.py
  - intera_interface/src/intera_interface/limb_ros2.py
  - exports in intera_interface/src/intera_interface/__init__.py
- Wired `LimbROS2` and `RobotEnableROS2` into launch-critical runtime script:
  - intera_examples/scripts/joint_position_joystick_ros2.py
  - added launch args (`limb`, `auto_enable`) in intera_examples/launch/joint_position_joystick.launch.py
- Started IO-layer ROS 2 runtime migration:
  - intera_interface/src/intera_io/io_command_ros2.py
  - intera_interface/src/intera_io/io_interface_ros2.py
  - intera_io package exports updated for ROS2-safe imports
  - intera_examples/scripts/gripper_joystick_ros2.py switched to IODeviceInterfaceROS2
- Added ROS2 head and navigator runtime baselines:
  - intera_interface/src/intera_interface/head_ros2.py
  - intera_interface/src/intera_interface/navigator_ros2.py
  - exported from intera_interface/src/intera_interface/__init__.py
  - example scripts: intera_examples/scripts/head_wobbler_ros2.py and intera_examples/scripts/navigator_io_ros2.py
- Added ROS2 camera runtime baseline:
  - intera_interface/src/intera_interface/camera_ros2.py
  - exported from intera_interface/src/intera_interface/__init__.py
  - example script: intera_examples/scripts/camera_display_ros2.py
- Updated intera_interface package init to guard ROS1-only imports, enabling ROS2 runtime imports in ROS2-only environments.
- Added ROS2 gripper and cuff runtime baselines:
  - intera_interface/src/intera_interface/gripper_ros2.py
  - intera_interface/src/intera_interface/cuff_ros2.py
  - exported from intera_interface/src/intera_interface/__init__.py
- Added ROS2 lights baseline and two gripper example ports:
  - intera_interface/src/intera_interface/lights_ros2.py
  - intera_examples/scripts/gripper_keyboard_ros2.py
  - intera_examples/scripts/gripper_cuff_control_ros2.py
  - intera_examples/src/intera_external_devices/__init__.py guarded for ROS2-safe getch import
- Added additional ROS2 example ports:
  - intera_examples/scripts/lights_blink_ros2.py
  - intera_examples/scripts/joint_trajectory_file_playback_ros2.py
- Ported remaining intera_interface utility executables to ROS2 counterparts:
  - intera_interface/scripts/enable_robot_ros2.py
  - intera_interface/scripts/home_joints_ros2.py
  - intera_interface/scripts/calibrate_arm_ros2.py
  - intera_interface/scripts/io_config_editor_ros2.py
  - intera_interface/scripts/send_urdf_fragment_ros2.py
- Updated launch-critical playback launcher to execute ROS2 playback script:
  - intera_examples/launch/joint_trajectory_file_playback.launch.py -> joint_trajectory_file_playback_ros2.py
- Updated intera_dataflow package init to guard ROS1 wait helper import and expose wait_for_ros2 in ROS2-only environments.
- Ran bounded host runtime smoke tests for ROS2 scripts:
  - head_wobbler_ros2: clean failure message due missing /robot/head/head_state
  - navigator_io_ros2: starts and runs, warns when navigator signals are unavailable
  - camera_display_ros2: clean failure message due missing robot_config.camera_config parameter
  - gripper_keyboard_ros2: clean failure message due missing /robot/state
  - gripper_cuff_control_ros2: starts and runs; warns when gripper configuration is unavailable
  - lights_blink_ros2: starts and fails cleanly when no light signals are present on host
  - joint_trajectory_file_playback_ros2: starts and fails cleanly if action server is unavailable
  - enable_robot_ros2: fails cleanly when /robot/state is unavailable
  - home_joints_ros2: exits cleanly on homing timeout when topics are unavailable
  - io_config_editor_ros2: fails cleanly when /io/end_effector/config is unavailable
  - send_urdf_fragment_ros2: fails fast on unreadable URDF path
  - calibrate_arm_ros2: exits cleanly when calibration action server is unavailable
- Verified ROS2 executable discovery for both packages:
  - intera_interface: calibrate_arm_ros2.py, enable_robot_ros2.py, home_joints_ros2.py, io_config_editor_ros2.py, joint_trajectory_action_server_ros2.py, send_urdf_fragment_ros2.py
  - intera_examples: camera_display_ros2.py, gripper_*_ros2.py, head_wobbler_ros2.py, joint_position_joystick_ros2.py, joint_trajectory_*_ros2.py, lights_blink_ros2.py, navigator_io_ros2.py
- Re-ran bridge smoke script on this host:
  - blocked by missing ROS1 commands (`roscore`, `rostopic`), so live bridge validation cannot complete here
- Revalidated staged visualization launch path:
  - `sawyer_description_rviz2.launch.py` runs headless (`start_rviz:=false`, `start_joint_state_publisher:=false`) and starts robot_state_publisher
  - optional joint_state_publisher path is blocked on this host because package `joint_state_publisher` is not installed

## Package Status Matrix

| Package | Current state | Next action |
| --- | --- | --- |
| intera_common/intera_core_msgs | ROS 2 interface metadata/build ported | Verify action/message/service generation with colcon |
| intera_common/intera_motion_msgs | ROS 2 interface metadata/build ported | Verify interfaces resolve dependency on intera_core_msgs |
| intera_common/intera_tools_description | ROS 2 metadata/build ported | Validate xacro and install tree in ROS 2 workspace |
| sawyer_robot/sawyer_description | ROS 2 metadata/build and launch ported | Validate full desktop RViz2 rendering on target machine |
| intera_sdk/intera_interface | ROS 2 metadata/build, launch, launch-critical runtime path, runtime-layer modules, baseline IO modules, and ROS2 utility executable ports completed | Continue motion-interface port, parameter/QoS hardening, and bridge-backed hardware validation |
| intera_sdk/intera_examples | ROS 2 metadata/build, launch, launch-critical runtime path, runtime-layer integration, and high-priority ROS2 examples added | Continue hardware parity validation and non-launch-critical example parity as needed |

## Correct Migration Gates

1. Interface generation gate
- Build message/action/service packages first.
- Do not begin rclpy API ports until all interfaces are generated in ROS 2.

2. Visualization gate
- Bring up robot description in RViz2 before hardware control ports.

3. Bridge gate (critical)
- Keep ROS 1 robot master path intact.
- Add ROS 1 to ROS 2 bridge/gateway for command and state topics/services.
- Validate read-only telemetry across bridge before command topics.

4. Safety gate
- Port robot enable/disable and status handling before arm motion commands.

5. Motion gate
- Port limb and motion stack after safety and IK/FK service parity is verified.

## Bridge Strategy (initial)

- Preferred short-term: dynamic ros1_bridge where message types are bridgeable.
- If custom interfaces prevent automatic bridging: implement explicit gateway nodes:
  - ROS 1 side node subscribes/publishes Sawyer-native ROS 1 topics.
  - ROS 2 side node exposes ROS 2-native APIs and translates payloads.

## Validation Checklist

- Interface build:
  - colcon build for intera_core_msgs and intera_motion_msgs
  - Source install setup and confirm generated interface artifacts exist
- Runtime smoke tests:
  - ROS 2 nodes can import generated message/service/action types
  - Bridge can relay at least one read-only robot state topic
  - Bridge smoke script currently reports missing prerequisites on this host: roscore, rostopic, ros1_bridge
- Safety tests:
  - Verify enable state and estop-related status propagation before motion commands

## Next file targets

- intera_sdk/intera_interface/src/intera_joint_trajectory_action/joint_trajectory_action.py
- intera_sdk/intera_interface/src/intera_io/io_interface.py
- intera_sdk/intera_interface/src/intera_io/io_command.py
- intera_sdk/intera_interface/src/intera_motion_interface/motion_trajectory.py
- intera_sdk/intera_interface/src/intera_motion_interface/motion_waypoint.py
- intera_sdk/intera_interface/src/intera_motion_interface/motion_controller_action_client.py
- intera_sdk/intera_interface/src/intera_interface/head.py
- intera_sdk/intera_interface/src/intera_interface/navigator.py
- intera_sdk/intera_interface/src/intera_interface/camera.py
- intera_sdk/intera_examples/scripts/camera_display.py
- intera_sdk/intera_interface/src/intera_interface/gripper.py
- intera_sdk/intera_interface/src/intera_interface/cuff.py
- intera_sdk/intera_interface/src/intera_interface/lights.py
