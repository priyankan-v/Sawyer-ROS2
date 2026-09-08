from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick = LaunchConfiguration("joystick")
    dev = LaunchConfiguration("dev")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "joystick",
                default_value="xbox",
                description="Joystick type (xbox, ps3, logitech).",
            ),
            DeclareLaunchArgument(
                "dev",
                default_value="/dev/input/js0",
                description="Joystick device path.",
            ),
            Node(
                package="joy",
                executable="joy_node",
                name="joy_node",
                output="screen",
                parameters=[{"dev": dev}],
            ),
            Node(
                package="intera_examples",
                executable="joint_position_joystick_ros2.py",
                name="rsdk_joint_position_joystick",
                output="screen",
                arguments=["--joystick", joystick],
            ),
        ]
    )
