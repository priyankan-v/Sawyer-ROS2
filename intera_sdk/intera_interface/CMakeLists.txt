cmake_minimum_required(VERSION 3.8)
project(intera_interface)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(intera_interface PACKAGE_DIR src/intera_interface)
ament_python_install_package(intera_control PACKAGE_DIR src/intera_control)
ament_python_install_package(intera_dataflow PACKAGE_DIR src/intera_dataflow)
ament_python_install_package(intera_io PACKAGE_DIR src/intera_io)
ament_python_install_package(intera_joint_trajectory_action PACKAGE_DIR src/intera_joint_trajectory_action)
ament_python_install_package(intera_motion_interface PACKAGE_DIR src/intera_motion_interface)

install(PROGRAMS
  scripts/home_joints.py
  scripts/joint_trajectory_action_server.py
  scripts/calibrate_arm.py
  scripts/io_config_editor.py
  scripts/send_urdf_fragment.py
  scripts/enable_robot.py

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
  DIRECTORY cfg/
  DESTINATION share/${PROJECT_NAME}/cfg
  USE_SOURCE_PERMISSIONS
)

install(
  DIRECTORY scripts/
  DESTINATION share/${PROJECT_NAME}/scripts
  USE_SOURCE_PERMISSIONS
)

ament_package()
