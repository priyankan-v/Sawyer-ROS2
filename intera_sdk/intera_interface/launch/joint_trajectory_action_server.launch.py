from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    limb = LaunchConfiguration("limb")
    mode = LaunchConfiguration("mode")
    stopped_velocity_tolerance = LaunchConfiguration("stopped_velocity_tolerance")

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
            DeclareLaunchArgument(
                "stopped_velocity_tolerance",
                default_value="0.05",
                description="Absolute max joint velocity (rad/s) used by final stopped-velocity tolerance check.",
            ),
            Node(
                package="intera_interface",
                executable="joint_trajectory_action_server_ros2.py",
                name="joint_trajectory_action_server",
                output="screen",
                arguments=[
                    "--limb",
                    limb,
                    "--mode",
                    mode,
                    "--stopped-velocity-tolerance",
                    stopped_velocity_tolerance,
                ],
            ),
        ]
    )
