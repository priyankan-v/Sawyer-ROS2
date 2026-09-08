# ROS 1 to ROS 2 Bridge Bring-up Plan

## Goal
Establish read-only telemetry flow from ROS 1 Sawyer topics into ROS 2 before enabling command paths.

## Initial telemetry scope
- /robot/joint_states (sensor_msgs/JointState)
- /robot/joint_limits (intera_core_msgs/JointLimits)
- /robot/limb/right/endpoint_state (intera_core_msgs/EndpointState)
- /robot/limb/right/tip_states (intera_core_msgs/EndpointStates)
- /robot/head/head_state (intera_core_msgs/HeadState)

## Preconditions
1. ROS 1 distro installed with roscore and rostopic.
2. ROS 2 distro installed (current repo is using Humble).
3. ros1_bridge installed in ROS 2 environment.
4. Matching interface packages available to both ROS 1 and ROS 2 sides for custom message bridging.

## First smoke test
Use the executable script:
- tools/bridge/ros1_ros2_bridge_smoke.sh

It performs:
1. Prerequisite checks.
2. roscore startup.
3. ros1_bridge dynamic bridge startup.
4. ROS 1 publication of sensor_msgs/JointState.
5. ROS 2 topic echo verification.

## Expected outputs
- Exit code 0: bridge path is operational for baseline telemetry type.
- Exit code 2: missing prerequisites (environment setup required).
- Exit code 1: bridge started but telemetry did not pass.

## Next bridge tests after baseline pass
1. Publish ROS 1 /robot/joint_states and verify ROS 2 subscriber receives it.
2. Verify custom intera_core_msgs topics bridge (requires matched message definitions).
3. Run read-only intera_interface ROS 2 probe against bridged topics.
4. Only after repeated read-only stability: begin gated command-topic experiments.
