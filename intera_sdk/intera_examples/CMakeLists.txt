cmake_minimum_required(VERSION 3.8)
project(intera_examples)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(intera_examples PACKAGE_DIR src/intera_examples)
ament_python_install_package(intera_external_devices PACKAGE_DIR src/intera_external_devices)

install(PROGRAMS
  scripts/ik_service_client.py
  scripts/go_to_joint_angles.py
  scripts/joint_position_waypoints.py
  scripts/joint_torque_springs.py
  scripts/gripper_cuff_control.py
  scripts/head_wobbler.py
  scripts/joint_recorder.py
  scripts/fk_service_client.py
  scripts/joint_velocity_wobbler.py
  scripts/navigator_io.py
  scripts/joint_position_joystick.py
  scripts/joint_position_keyboard.py
  scripts/gripper_keyboard.py
  scripts/stop_motion_trajectory.py
  scripts/go_to_joint_angles_in_contact.py
  scripts/joint_position_file_playback.py
  scripts/go_to_cartesian_pose.py
  scripts/head_display_image.py
  scripts/joint_trajectory_client.py
  scripts/joint_trajectory_client_ros2.py
  scripts/joint_trajectory_file_playback.py
  scripts/set_interaction_options.py
  scripts/constrained_zeroG.py
  scripts/gripper_joystick.py
  scripts/gripper_joystick_ros2.py
  scripts/joint_position_joystick_ros2.py
  scripts/lights_blink.py
  scripts/camera_display.py
  scripts/send_random_trajectory.py
  scripts/send_traj_from_file.py

  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY scripts/
  DESTINATION lib/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY share/
  DESTINATION share/${PROJECT_NAME}/share
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
  USE_SOURCE_PERMISSIONS
)

ament_package()
