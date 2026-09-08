# Real Sawyer Enablement Next Steps for the Migrated ROS2 Codebase

This document explains what still must be done to make the physical Sawyer robot work with the migrated ROS2 codebase.

It is not a migration-code to-do list.
It is the real-robot enablement checklist after the code migration.

## 1) Core Reality

The migrated codebase now provides ROS2-native packages, launch files, interfaces, and example scripts.

But the physical Sawyer robot is still ROS1-native at the communication boundary used by the original SDK.

That means the remaining work is mostly about:
- environment setup
- ROS1 and ROS2 coexistence
- bridge or gateway bring-up
- robot connectivity
- live validation on hardware
- safety verification before motion

## 2) What You Still Need To Do

## 2.1 Install and prepare a real bridge environment

You need a machine that can participate in both sides:
- ROS1 side for Sawyer communication
- ROS2 side for the migrated code

Minimum requirements:
- ROS1 installed with Sawyer-compatible tools
- ROS2 Humble installed
- `ros1_bridge` installed and usable
- ROS1 tools available: `roscore`, `rostopic`, `rosservice`, `rosnode`
- ROS2 tools available: `ros2`, `colcon`

If you plan to use one machine for both, the environment sourcing order and shell setup must be controlled carefully.

## 2.2 Replace the old ROS1-only shell workflow

The original workflow depended on `intera.sh` and ROS1 environment assumptions.

You still need to create a production-ready replacement workflow for:
- sourcing ROS1 environment
- sourcing ROS2 environment
- setting robot IP / `ROS_MASTER_URI` / `ROS_IP` or `ROS_HOSTNAME`
- starting the bridge safely
- sourcing the ROS2 workspace overlay

What you should create:
- one documented ROS1 shell entrypoint for robot communication
- one documented ROS2 shell entrypoint for the migrated workspace
- one bridge launch/startup script or guide that joins them correctly

Without this, operators will start processes in the wrong environment and the system will fail intermittently.

## 2.3 Verify direct ROS1 communication with Sawyer first

Before testing any ROS2 node, confirm the old communication path still works.

You need to verify from the ROS1 side:
- the machine can reach the robot network
- `roscore` / master connectivity works
- Sawyer topics are visible
- Sawyer services are visible
- Sawyer action endpoints are visible

Do not start with ROS2 testing first.
If the ROS1 side is not healthy, bridge debugging is wasted time.

## 2.4 Bring up the ROS1 to ROS2 bridge

Once ROS1 communication works, start the bridge.

Goal of the bridge stage:
- expose robot state into ROS2
- expose command topics/services/actions into ROS2
- confirm message translation works for the migrated interface set

Minimum validation goals:
- a bridged robot state topic appears in ROS2
- at least one read-only topic is observable in ROS2
- at least one command topic can be published from ROS2 and observed on the ROS1 side
- action/service flows required by migrated scripts are bridged correctly

You must especially validate the interfaces used by the migrated code:
- `/robot/state`
- `/robot/joint_states`
- `/robot/head/head_state`
- `/robot/limb/<limb>/joint_command`
- `/robot/limb/<limb>/follow_joint_trajectory`
- `/io/end_effector/config`
- `/io/end_effector/<device>/state`
- `/io/robot/cuff/*`
- `/io/internal_camera/*`
- IK/FK service endpoints
- calibration action endpoint if calibration is required

If automatic dynamic bridge behavior is insufficient for custom messages/actions, you will need explicit gateway nodes.

## 2.5 Decide whether to use dynamic bridge or explicit gateway nodes

This is an architecture decision you still need to finalize.

Option A: dynamic `ros1_bridge`
- Faster to bring up.
- Good if all required message/action/service types bridge cleanly.
- Lower implementation effort.

Option B: explicit gateway nodes
- More work.
- Better when custom interfaces or actions do not bridge reliably.
- Better if you want tighter control over safety checks, filtering, throttling, or field translation.

If any of these fail under dynamic bridge, move to explicit gateways early instead of fighting implicit bridge behavior:
- FollowJointTrajectory action path
- custom Intera messages/actions
- IO configuration/state topics
- calibration command action

## 2.6 Install any missing ROS2 runtime packages on the control host

The migrated code depends on several ROS2 packages beyond base Python.

You still need to ensure the actual operator host has all runtime dependencies installed, including at least:
- `xacro`
- `rviz2`
- `joint_state_publisher`
- `joint_state_publisher_gui` if used
- `cv_bridge`
- joystick support packages if joystick control is needed

If camera, RViz, joystick, or optional launch flows are part of operations, verify those dependencies explicitly on the target machine, not only on the dev host.

## 2.7 Validate the migrated ROS2 package set on the target machine

On the actual control machine, you still need to repeat the package validation steps:
- build selected ROS2 packages
- source the overlay
- confirm `ros2 pkg executables` lists expected scripts
- confirm launch files parse
- confirm imported packages resolve in the operator environment

This prevents environment drift between development and deployment systems.

## 2.8 Run read-only validation on the real robot before any motion

Before enabling motion, verify the following from ROS2:
- `enable_robot_ros2.py -s` returns live state
- `navigator_io_ros2.py` sees real signals
- `camera_display_ros2.py` sees real camera configuration and streams
- `head_wobbler_ros2.py` can read head state
- `io_config_editor_ros2.py` can read EE config if end effector is attached
- `gripper_*_ros2.py` scripts can discover gripper state if gripper is present

This is the first real proof that the bridge and topic graph are working correctly.

## 2.9 Validate safety controls before any trajectory motion

You must verify the migrated safety-related paths first.

Required checks:
- enable works
- disable works
- stop works
- reset works when appropriate
- robot state transitions are visible in ROS2
- E-stop behavior is understood and not masked by bridge behavior

This is mandatory before any arm motion test.

## 2.10 Validate IK/FK and limb state paths

Before trajectory execution, verify:
- joint states arrive continuously
- limb state is readable through `LimbROS2`
- endpoint state is valid if required by your application
- tip states are present if required
- IK service requests succeed
- FK service requests succeed

If these are broken, motion examples may start but behave incorrectly.

## 2.11 Validate gripper, cuff, lights, and camera behavior on hardware

These features were only host-smoke tested locally.

You still need robot-side validation for:
- gripper command response
- cuff button signal mapping
- light signal naming and light control behavior
- camera stream control and callbacks
- gripper calibration/error/reboot flows

Expect small naming or timing mismatches here; these are common late-stage parity issues.

## 2.12 Validate trajectory action path on real hardware

This is the most important motion gate.

Test order should be:
1. Start ROS2 trajectory action server.
2. Confirm the action endpoint exists in ROS2.
3. Run a very small-amplitude trajectory with `joint_trajectory_client_ros2.py`.
4. Confirm result codes and final tolerances are correct.
5. Confirm cancel behavior works.
6. Confirm stop/disable recovery works.
7. Then test `joint_trajectory_file_playback_ros2.py` with a conservative recorded file.

Do not start with large file playback.

## 2.13 Validate joystick and operator-control workflows

If joystick control is part of actual use, test all of these on target hardware:
- joystick device discovery
- correct mapping for the actual controller model
- joint position joystick control
- gripper joystick control
- robot enable behavior with `auto_enable`
- safe operator stop flow

The joystick stack may be operational in software but still fail in practice due to device names, permission issues, or controller mapping mismatches.

## 2.14 Decide what to do with the unmigrated motion-interface stack

The larger `intera_motion_interface` stack is still not fully ported as a production ROS2-native path.

You still need to decide one of these:
- fully port it to ROS2
- replace it with action/topic-based workflows already migrated
- keep it ROS1-only behind bridge/gateway logic if acceptable

This matters if your real robot workflow depends on:
- motion trajectories loaded from higher-level motion interface abstractions
- waypoint-based motion planning APIs
- interaction options and related motion-control helpers

If you need those features, the migration is not operationally complete until this decision is implemented.

## 2.15 Complete parameter, QoS, and executor hardening

The current ROS2 migration is a solid baseline, but production robot operation still needs a hardening pass for:
- explicit QoS policies
- executor model selection
- callback timing assumptions
- parameter declaration and loading strategy
- startup ordering and timeout tuning
- recovery behavior after communication loss

This is not optional if the robot will be used repeatedly by operators.

## 2.16 Create a final deployment and operations runbook

Before handing this to real users, write an operator-focused runbook that includes:
- exact shell setup sequence
- bridge startup steps
- build/source commands
- robot enable procedure
- read-only checks
- motion test sequence
- stop/reset recovery procedure
- known failure modes and fixes

The codebase now has technical migration notes, but real robot use also needs an operator runbook.

## 3) What Is Already Done

These parts are already in place in the migrated codebase:
- ROS2 package metadata/build for core migrated packages
- ROS2 message/action/service generation packages
- ROS2 trajectory action server/client baseline
- ROS2 runtime interfaces for robot enable, limb, IO, head, navigator, camera, gripper, cuff, lights
- ROS2 utility script counterparts under `intera_interface/scripts`
- ROS2 example script counterparts for major operator flows
- launch-file conversion for launch-critical paths
- host-side build, launch, import, and smoke validation

So the remaining work is not mostly coding from scratch.
It is mostly bridge, hardware validation, environment hardening, and final parity closure.

## 4) Short Practical Order To Finish Real-Robot Enablement

Do these in this order:

1. Build a proper ROS1 + ROS2 bridge environment.
2. Verify direct ROS1 communication to Sawyer.
3. Bring up and validate the ROS1 to ROS2 bridge.
4. Verify read-only robot state in ROS2.
5. Verify safety controls in ROS2 (`state`, `enable`, `disable`, `stop`, `reset`).
6. Verify limb state and IK/FK services.
7. Verify head, navigator, camera, gripper, cuff, and lights on hardware.
8. Validate trajectory action path with small conservative commands.
9. Validate file playback and joystick workflows.
10. Finish motion-interface strategy if your production workflow still depends on it.
11. Perform QoS/executor/parameter hardening.
12. Write the final deployment runbook and acceptance checklist.

## 5) Definition of Done for the Real Sawyer

You can treat the migrated codebase as actually ready for the real robot only when all of the following are true:
- ROS1 communication to Sawyer is healthy.
- ROS1 to ROS2 bridge is stable.
- ROS2 read-only robot state is confirmed.
- safety controls are validated from ROS2.
- limb, IK, FK, head, navigator, camera, IO, gripper, cuff, and lights behave correctly on hardware.
- trajectory action server/client path works on robot.
- file playback works conservatively and repeatably.
- joystick/operator workflows are tested on the actual control host.
- remaining motion-interface dependencies are resolved.
- deployment/runbook documentation is complete.

Until then, the migration is technically advanced and well validated, but not yet fully operational on the physical Sawyer.
