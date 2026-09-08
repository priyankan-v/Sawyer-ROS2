# ROS 2 Launch Ports

These launch files are ROS 2 ports of the original ROS 1 XML launch files.

Files:
- gripper_joystick.launch.py
- joint_position_joystick.launch.py
- joint_trajectory_client.launch.py
- joint_trajectory_file_playback.launch.py

Prerequisites:
- intera_examples and intera_interface must be ported to ROS 2 ament before these launches are runnable with `ros2 launch`.
- `joy` package must be available for joystick launch variants.

Validation strategy:
1. Verify syntax with `ros2 launch <pkg> <launch.py> --show-args`.
2. Start in headless/read-only mode where possible.
3. Introduce hardware command paths only after bridge and safety gates are complete.
