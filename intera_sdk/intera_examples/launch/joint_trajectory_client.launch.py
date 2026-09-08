from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    limb = LaunchConfiguration("limb")

    action_server_launch = PathJoinSubstitution(
        [
            FindPackageShare("intera_interface"),
            "launch",
            "joint_trajectory_action_server.launch.py",
        ]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "limb",
                default_value="right",
                description="Target limb name.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(action_server_launch),
                launch_arguments={"limb": limb, "mode": "position"}.items(),
            ),
            Node(
                package="intera_examples",
                executable="joint_trajectory_client_ros2.py",
                name="sdk_joint_trajectory_test",
                output="screen",
                arguments=["--limb", limb],
            ),
        ]
    )
