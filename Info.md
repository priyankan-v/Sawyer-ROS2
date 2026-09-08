Sawyer ROS 2 Migration Notes

ROS 1 dependencies currently used by the SDK:
git-core python-argparse python-wstool python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter

Current ROS 1 SDK bootstrap:
./intera.sh

Critical architecture constraint:
intera.sh sets ROS_MASTER_URI toward Sawyer, so the workstation SDK talks to the robot's ROS 1 master. Converting rospy to rclpy alone does not make physical Sawyer ROS 2-native. A ROS 1 to ROS 2 bridge/gateway stage is required before commanding hardware from ROS 2 nodes.

Migration execution order (safety-first):

Phase 0 - Interface and build foundation
[x] Port intera_core_msgs
[x] Port intera_motion_msgs
[x] Port intera_tools_description
[x] Port sawyer_description package metadata/build files
[x] Validate colcon build for first package batch
[ ] Validate Sawyer visualization in RViz2

Phase 1 - ROS 1/ROS 2 coexistence
[ ] Establish ROS 1 to ROS 2 real-robot communication
[ ] Replace/redesign intera.sh for dual-environment setup
[ ] Replace .rosinstall/wstool workflow with .repos/vcs

Phase 2 - Low-risk software ports
[ ] Port utility modules
[ ] Port read-only robot interfaces
[ ] Add QoS, executor, timing, shutdown, and parameter design

Phase 3 - Device and control interfaces
[ ] Port I/O, gripper, head, navigator, camera
[ ] Port safety/robot enable functionality
[ ] Port IK/FK services
[ ] Port limb.py in stages
[ ] Introduce position control
[ ] Port the motion interface

Phase 4 - Actions, parameters, and execution
[ ] Port ROS 1 actionlib to ROS 2 actions
[ ] Replace dynamic reconfigure with ROS 2 parameters
[ ] Port the joint trajectory action server
[ ] Port every intera_interface/scripts executable
[ ] Port ROS 1 launch files

Phase 5 - Examples and validation
[ ] Port intera_examples in increasing order of hardware risk
[ ] Run staged simulation, mock, read-only, and hardware tests

Immediate next work items:
1) Validate Sawyer visualization path in RViz2 (xacro + robot_state_publisher + rviz2).
2) Port URDF and meshes packages for RViz2 visualization without hardware motion.
3) Stand up a ros1_bridge or custom gateway path for real robot communications.
