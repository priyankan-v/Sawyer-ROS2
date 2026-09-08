from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick = LaunchConfiguration("joystick")
    dev = LaunchConfiguration("dev")
    limb = LaunchConfiguration("limb")
    auto_enable = LaunchConfiguration("auto_enable")

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
            DeclareLaunchArgument(
                "limb",
                default_value="right",
                description="Target limb name.",
            ),
            DeclareLaunchArgument(
                "auto_enable",
                default_value="false",
                description="Enable robot on startup (true/false).",
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
                arguments=["--joystick", joystick, "--limb", limb, "--auto-enable", auto_enable],
            ),
        ]
    )
