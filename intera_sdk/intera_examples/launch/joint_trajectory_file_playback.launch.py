from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    file_path = LaunchConfiguration("file_path")
    loops = LaunchConfiguration("loops")

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
                "file_path",
                description="Path to joint trajectory file.",
            ),
            DeclareLaunchArgument(
                "loops",
                default_value="1",
                description="Number of playback loops.",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(action_server_launch),
                launch_arguments={"mode": "velocity"}.items(),
            ),
            Node(
                package="intera_examples",
                executable="joint_trajectory_file_playback_ros2.py",
                name="sdk_joint_trajectory_file_playback",
                output="screen",
                arguments=["--file", file_path, "--number_loops", loops],
            ),
        ]
    )
