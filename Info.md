dependencies: git-core python-argparse python-wstool python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter

SDK Installation: git-core python-argparse python-wstool python-vcstools python-rosdep ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter

Initialize your SDK environment: ./intera.sh

the above are used and needed for operate Sawyer robot.

Primary goal is to move this code base to ros 2.
the following are the tasks need to be done to achieve this goal.

"A particularly important conclusion is built into the plan: the current intera.sh sets ROS_MASTER_URI toward Sawyer, so the Intera workstation SDK is communicating with a ROS 1 master on the robot. Therefore, simply converting rospy → rclpy will not make the physical Sawyer a ROS 2 robot. The plan deliberately introduces a ROS 1/ROS 2 bridge or gateway stage before trying to command the real arm."

Create the ROS 2 workspace and migration branches
Port intera_core_msgs
Port intera_motion_msgs
Port intera_tools_description
Port sawyer_description and get Sawyer into RViz2
Establish ROS 1 ↔ ROS 2 real-robot communication
Port utility modules
Port read-only robot interfaces
Port I/O, gripper, head, navigator, camera
Port safety/robot enable functionality
Port limb.py in stages
Port IK/FK services
Introduce position control
Port the motion interface
Port ROS 1 actionlib to ROS 2 actions
Replace dynamic reconfigure with ROS 2 parameters
Port the joint trajectory action server
Port every intera_interface/scripts executable
Replace/redesign intera.sh
Port intera_examples in increasing order of hardware risk
Port ROS 1 launch files
Replace .rosinstall/wstool workflow with .repos/vcs
Add QoS, executor, timing, shutdown, and parameter design
Run staged simulation/mock/read-only/hardware tests
