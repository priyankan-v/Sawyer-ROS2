from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    limb = LaunchConfiguration("limb")
    mode = LaunchConfiguration("mode")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "limb",
                default_value="right",
                description="Target limb name.",
            ),
            DeclareLaunchArgument(
                "mode",
                default_value="position",
                description="Control mode for trajectory server (position or velocity).",
            ),
            Node(
                package="intera_interface",
                executable="joint_trajectory_action_server.py",
                name="joint_trajectory_action_server",
                output="screen",
                arguments=["--limb", limb, "--mode", mode],
            ),
        ]
    )
