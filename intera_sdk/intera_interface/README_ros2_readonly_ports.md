# ROS 2 Read-Only Interface Ports (Initial)

This file tracks the first read-only ROS 2 ports for intera_interface.

Implemented modules:
- src/intera_interface/robot_params_ros2.py
- src/intera_interface/joint_limits_ros2.py
- src/intera_dataflow/wait_for_ros2.py
- scripts/ros2_readonly_probe.py

Notes:
- These are additive ports and do not modify ROS 1 modules in place.
- RobotParamsROS2 expects node-scoped ROS 2 parameters with ROS 1-style names mapped to dotted names.
- JointLimitsROS2 listens to /robot/joint_limits and waits for first message before returning.

Next read-only candidates:
- head.py state-only accessors
- navigator.py read-only signals
- camera.py topic enumeration and state path
