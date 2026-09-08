# ROS2 Migration File Change Rationale

This document explains why each migration file was changed, what risk it addresses, and how it was validated.

## How to read this document

For each file:
- What changed: concrete implementation delta.
- Why it changed: migration requirement or bug/risk addressed.
- Validation: the checks used to verify behavior.

## A) Planning and Tracking Files

### Info.md
- What changed:
  - Updated phase checklists as ports were completed.
  - Marked completed runtime/example/script migration milestones.
  - Refreshed remaining work to bridge/hardware/parity hardening tasks.
- Why it changed:
  - Keep migration execution order and completion state accurate.
  - Avoid stale planning that misstates completed engineering work.
- Validation:
  - Manual consistency check against implemented files and successful build/smoke outputs.

### ROS2_MIGRATION_TRACKER.md
- What changed:
  - Added completed milestones for runtime-layer ports and script ports.
  - Added host smoke outcomes (clean failures/warnings for missing hardware topics).
  - Updated package status matrix and next file targets.
- Why it changed:
  - Provide auditable migration evidence and realistic blockers.
  - Preserve exact gate status for handoff and future continuation.
- Validation:
  - Cross-checked entries against command results and file diffs.

## B) Build and Install Wiring

### intera_sdk/intera_examples/CMakeLists.txt
- What changed:
  - Added install entries for new ROS2 scripts:
    - camera_display_ros2.py
    - head_wobbler_ros2.py
    - navigator_io_ros2.py
    - gripper_keyboard_ros2.py
    - gripper_cuff_control_ros2.py
    - lights_blink_ros2.py
    - joint_trajectory_file_playback_ros2.py
- Why it changed:
  - `ros2 run` only works for installed executables.
  - Launch files and operator workflows depend on these entrypoints.
- Validation:
  - `colcon build --packages-select intera_examples`
  - `ros2 pkg executables intera_examples`

### intera_sdk/intera_interface/CMakeLists.txt
- What changed:
  - Added install entries for new ROS2 utility scripts:
    - enable_robot_ros2.py
    - home_joints_ros2.py
    - calibrate_arm_ros2.py
    - io_config_editor_ros2.py
    - send_urdf_fragment_ros2.py
- Why it changed:
  - Close Phase 4 script port gap while preserving ROS1 scripts in parallel.
- Validation:
  - `colcon build --packages-select intera_interface`
  - `ros2 pkg executables intera_interface`

### intera_sdk/intera_interface/package.xml
- What changed:
  - Added dependency required by ROS2 behavior (`tf2_ros`) for runtime compatibility.
- Why it changed:
  - Keep ROS2 package dependency graph explicit and build/runtime safe.
- Validation:
  - Build/install success for intera_interface in ROS2 selected-package builds.

## C) Launch Conversion and Wiring

### intera_sdk/intera_examples/launch/joint_position_joystick.launch.py
- What changed:
  - Added `limb` and `auto_enable` launch arguments.
  - Passed new arguments into ROS2 joystick node.
- Why it changed:
  - Support runtime-layer integration with LimbROS2 and RobotEnableROS2.
- Validation:
  - `ros2 launch intera_examples joint_position_joystick.launch.py --show-args`

### intera_sdk/intera_examples/launch/joint_trajectory_file_playback.launch.py
- What changed:
  - Switched executable from ROS1 playback script to ROS2 playback script (`joint_trajectory_file_playback_ros2.py`).
- Why it changed:
  - Ensure launch path is fully ROS2-native for trajectory playback.
- Validation:
  - `ros2 launch intera_examples joint_trajectory_file_playback.launch.py --show-args`

## D) New ROS2 Example Scripts

### intera_sdk/intera_examples/scripts/camera_display_ros2.py
- What changed:
  - ROS2 camera display example using CamerasROS2 + RobotParamsROS2.
  - Supports raw/rectified image options and optional edge mode.
- Why it changed:
  - Replace ROS1 camera example path for ROS2 runtime testing and operations.
- Validation:
  - CLI help smoke and host runtime smoke (clean exit when camera params absent).

### intera_sdk/intera_examples/scripts/head_wobbler_ros2.py
- What changed:
  - ROS2 head wobble example using HeadROS2 + RobotEnableROS2.
  - Added startup exception handling for missing head topics.
- Why it changed:
  - Provide ROS2 head-control exercise path with robust host behavior.
- Validation:
  - Host runtime smoke: clear timeout/error messaging without crash.

### intera_sdk/intera_examples/scripts/navigator_io_ros2.py
- What changed:
  - ROS2 navigator callback example with signal registration and timed spin.
- Why it changed:
  - Port ROS1 navigator read-only example to ROS2.
- Validation:
  - Runtime smoke: starts, warns when signals absent, no unhandled crash.

### intera_sdk/intera_examples/scripts/gripper_joystick_ros2.py
- What changed:
  - Replaced direct JSON command pub path with IODeviceInterfaceROS2 abstraction.
- Why it changed:
  - Unify command transport through migrated ROS2 IO layer and reduce duplication.
- Validation:
  - Build/syntax and launch/runtime smoke for joystick stack.

### intera_sdk/intera_examples/scripts/gripper_keyboard_ros2.py
- What changed:
  - New ROS2 keyboard gripper controller with bindings for open/close/calibrate/reboot/velocity/position/stop.
  - Added graceful handling for missing robot state and missing gripper.
- Why it changed:
  - Port an operator-facing utility script critical for gripper bring-up.
- Validation:
  - CLI help pass.
  - Runtime smoke: explicit, clean host-gated failures.

### intera_sdk/intera_examples/scripts/gripper_cuff_control_ros2.py
- What changed:
  - New ROS2 cuff button to gripper control flow.
  - Optional lights path and resilient shutdown behavior.
- Why it changed:
  - Port cuff-driven gripper interaction while supporting no-light/no-gripper scenarios.
- Validation:
  - CLI help pass.
  - Runtime smoke: starts, warns on missing gripper config, no shutdown traceback.

### intera_sdk/intera_examples/scripts/lights_blink_ros2.py
- What changed:
  - New ROS2 light blink example using LightsROS2.
- Why it changed:
  - Port legacy lights test utility to ROS2 for signal-path validation.
- Validation:
  - CLI help pass.
  - Runtime smoke: clean error if requested light signal not present.

### intera_sdk/intera_examples/scripts/joint_position_joystick_ros2.py
- What changed:
  - Integrated RobotEnableROS2 and LimbROS2.
  - Added robust handling for missing robot state topic on host.
- Why it changed:
  - Move launch-critical joystick control from ad hoc pub/sub to runtime-layer APIs.
- Validation:
  - Launch arg validation and package build success.

### intera_sdk/intera_examples/scripts/joint_trajectory_client_ros2.py
- What changed:
  - ROS2 action client baseline for FollowJointTrajectory.
- Why it changed:
  - Provide ROS2 client counterpart for migrated action server.
- Validation:
  - Build and CLI/launch smoke validation.

### intera_sdk/intera_examples/scripts/joint_trajectory_file_playback_ros2.py
- What changed:
  - New ROS2 trajectory CSV playback script with action-based limb control and optional gripper track playback.
  - Added early file-path validation for immediate user feedback.
- Why it changed:
  - Port trajectory playback to ROS2 and preserve operational behavior.
- Validation:
  - CLI help pass.
  - Runtime smoke: clear fast-fail for missing file and clean action-server timeout behavior.

## E) New ROS2 intera_interface Utility Scripts

### intera_sdk/intera_interface/scripts/enable_robot_ros2.py
- What changed:
  - ROS2 utility for state/enable/disable/reset/stop actions using RobotEnableROS2.
- Why it changed:
  - Replace ROS1 CLI utility for safety operations in ROS2 workflows.
- Validation:
  - CLI help pass.
  - Runtime smoke: clean error when /robot/state is absent.

### intera_sdk/intera_interface/scripts/home_joints_ros2.py
- What changed:
  - ROS2 joint homing utility with timeout and optional pre-enable path.
- Why it changed:
  - Port homing script used in bring-up and recovery workflows.
- Validation:
  - CLI help pass.
  - Runtime smoke: clean timeout path on host without live homing topics.

### intera_sdk/intera_interface/scripts/calibrate_arm_ros2.py
- What changed:
  - ROS2 calibration action client using intera_core_msgs CalibrationCommand action.
  - Limb neutral move and gripper-attachment guard retained in ROS2 style.
- Why it changed:
  - Port calibration workflow and maintain safety-related operator flow.
- Validation:
  - CLI help pass.
  - Runtime smoke: clean timeout when calibration action server unavailable.

### intera_sdk/intera_interface/scripts/io_config_editor_ros2.py
- What changed:
  - ROS2 end-effector config save/load utility on `/io/end_effector/*` topics.
- Why it changed:
  - Replace ROS1 ClickSmart config helper with ROS2-capable tooling.
- Validation:
  - CLI help pass.
  - Runtime smoke: clean error when EE config topic is unavailable.

### intera_sdk/intera_interface/scripts/send_urdf_fragment_ros2.py
- What changed:
  - ROS2 URDF fragment sender with xacro processing compatibility and path validation.
- Why it changed:
  - Port URDF fragment injection helper used in end-effector modeling workflows.
- Validation:
  - CLI help pass.
  - Runtime smoke: immediate, clear error for unreadable file.

### intera_sdk/intera_interface/scripts/joint_trajectory_action_server_ros2.py
- What changed:
  - ROS2 FollowJointTrajectory action server with goal validation, path tolerance checks, goal/stopped-velocity tolerance checks.
  - Added graceful shutdown handling for timeout-driven termination (`ExternalShutdownException` + guarded `rclpy.shutdown()`).
- Why it changed:
  - Core launch-critical motion control path in ROS2.
  - Shutdown hardening prevents false-negative traceback noise during test harness timeouts.
- Validation:
  - Build + launch + runtime startup smoke.
  - Regression retest after shutdown fix confirms clean timeout termination.

## F) Runtime-Layer ROS2 Interface Modules

### intera_sdk/intera_interface/src/intera_interface/robot_enable_ros2.py
- What changed:
  - ROS2 wrapper around robot state/enable/reset/stop and version checks.
- Why it changed:
  - Safety gate before higher-risk motion commands.
- Validation:
  - Consumed by migrated scripts and smoke-tested via enable utility.

### intera_sdk/intera_interface/src/intera_interface/limb_ros2.py
- What changed:
  - ROS2 limb command/state interface with IK/FK service clients and configurable startup requirements.
- Why it changed:
  - Replace direct script-level topic command logic with reusable runtime abstraction.
- Validation:
  - Used by joystick/playback/calibration paths; build/smoke validated.

### intera_sdk/intera_interface/src/intera_interface/head_ros2.py
- What changed:
  - ROS2 head pan interface with wait-for-state behavior and set_pan command path.
- Why it changed:
  - Port head control baseline for examples and applications.
- Validation:
  - head_wobbler_ros2 smoke behavior.

### intera_sdk/intera_interface/src/intera_interface/navigator_ros2.py
- What changed:
  - ROS2 navigator signal polling/callback registration.
- Why it changed:
  - Port navigator read-only interaction path.
- Validation:
  - navigator_io_ros2 runtime smoke.

### intera_sdk/intera_interface/src/intera_interface/camera_ros2.py
- What changed:
  - ROS2 camera configuration/state/stream control and callback subscription API.
- Why it changed:
  - Port camera operations to ROS2 while preserving operator utility functions.
- Validation:
  - camera_display_ros2 runtime smoke.

### intera_sdk/intera_interface/src/intera_interface/gripper_ros2.py
- What changed:
  - ROS2 gripper control API with calibration and command helpers.
- Why it changed:
  - Port gripper control foundation for joystick/keyboard/cuff examples.
- Validation:
  - Used in multiple examples; host-gated runtime checks clean.

### intera_sdk/intera_interface/src/intera_interface/cuff_ros2.py
- What changed:
  - ROS2 cuff signal access with callback registration.
- Why it changed:
  - Port cuff interaction primitives for cuff-control example.
- Validation:
  - Used by gripper_cuff_control_ros2 and smoke-validated.

### intera_sdk/intera_interface/src/intera_interface/lights_ros2.py
- What changed:
  - ROS2 lights interface backed by IODeviceInterfaceROS2.
- Why it changed:
  - Needed for lights example and optional cuff-control indicator path.
- Validation:
  - lights_blink_ros2 and gripper_cuff_control_ros2 no-lights path.

### intera_sdk/intera_interface/src/intera_interface/robot_params_ros2.py
- What changed:
  - Logging calls updated to rclpy-safe string formatting.
- Why it changed:
  - Remove rospy-style logger argument usage that caused runtime TypeError.
- Validation:
  - Confirmed by repeated runtime startup of dependent scripts.

## G) IO/Dataflow Packaging and ROS2 IO Layer

### intera_sdk/intera_interface/src/intera_io/io_command_ros2.py
- What changed:
  - ROS2 IO command container helpers for signal/port set operations.
- Why it changed:
  - Shared command serialization utility for ROS2 IO interfaces.
- Validation:
  - Used by IODeviceInterfaceROS2 and dependent scripts.

### intera_sdk/intera_interface/src/intera_io/io_interface_ros2.py
- What changed:
  - ROS2 IO interface and device abstractions with config/state subscriptions and command publishing.
- Why it changed:
  - Foundational ROS2 replacement for rospy-based IO layer.
- Validation:
  - Used across gripper/navigator/cuff/camera/lights script paths.

### intera_sdk/intera_interface/src/intera_io/__init__.py
- What changed:
  - Added ROS1 import guards and explicit ROS2 IO exports.
- Why it changed:
  - Prevent ROS1 import failures from breaking ROS2-only execution.
- Validation:
  - ROS2 scripts import intera_io safely in ROS2-only environment.

### intera_sdk/intera_interface/src/intera_dataflow/__init__.py
- What changed:
  - Guarded ROS1 wait_for import and exported wait_for_ros2.
- Why it changed:
  - Removed runtime blocker where rospy import failed in ROS2 scripts.
- Validation:
  - Confirmed by successful import/start of ROS2 nodes using wait_for_ros2.

## H) ROS2-Safe Package Exports

### intera_sdk/intera_interface/src/intera_interface/__init__.py
- What changed:
  - Guarded ROS1-only imports.
  - Exported ROS2 classes (HeadROS2, NavigatorROS2, CamerasROS2, GripperROS2, CuffROS2, LightsROS2, LimbROS2, RobotEnableROS2).
- Why it changed:
  - Mixed ROS1/ROS2 package needed import safety and explicit ROS2 access points.
- Validation:
  - ROS2 scripts import `intera_interface` successfully in ROS2-only host.

### intera_sdk/intera_examples/src/intera_external_devices/__init__.py
- What changed:
  - Guarded ROS1 joystick imports while keeping `getch` available.
- Why it changed:
  - Prevent `rospy` dependency from breaking keyboard-based ROS2 scripts.
- Validation:
  - gripper_keyboard_ros2 import/startup path successful.

## I) Test Evidence Summary

The following evidence was used repeatedly throughout migration:
- Selected-package ROS2 builds succeeded (`intera_core_msgs`, `intera_motion_msgs`, `intera_tools_description`, `sawyer_description`, `intera_interface`, `intera_examples`).
- Launch `--show-args` succeeded for migrated launch targets.
- ROS2 executable discovery includes all migrated scripts.
- Full help-mode smoke across all `*ros2.py` executables succeeded.
- Runtime smoke tests exercised no-hardware paths and confirmed explicit, clean failure behavior.
- One real runtime bug was found (action server shutdown traceback on timeout) and fixed in `joint_trajectory_action_server_ros2.py`.

## J) Residual Risks and Non-Code Blockers

These are not unresolved coding tasks in this pass; they are environment or scope blockers:
- Live bridge gate requires ROS1 tooling (`roscore`, `rostopic`) and configured `ros1_bridge`.
- Full hardware confirmation requires active Sawyer topics/actions/services.
- Optional launch path with `joint_state_publisher` requires package installation on host.
- Full motion-interface ROS2 parity remains a separate, larger subsystem migration task.
