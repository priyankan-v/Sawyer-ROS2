from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pedestal = LaunchConfiguration("pedestal")
    electric_gripper = LaunchConfiguration("electric_gripper")
    start_joint_state_publisher = LaunchConfiguration("start_joint_state_publisher")
    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    start_rviz = LaunchConfiguration("start_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    xacro_path = PathJoinSubstitution(
        [FindPackageShare("sawyer_description"), "urdf", "sawyer.urdf.xacro"]
    )

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_path,
                " gazebo:=false",
                " static:=true",
                " pedestal:=",
                pedestal,
                " electric_gripper:=",
                electric_gripper,
            ]
        )
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "pedestal",
                default_value="true",
                description="Whether to include the pedestal in the model.",
            ),
            DeclareLaunchArgument(
                "electric_gripper",
                default_value="false",
                description="Whether to include the electric gripper model.",
            ),
            DeclareLaunchArgument(
                "start_joint_state_publisher",
                default_value="false",
                description="Start joint_state_publisher node(s).",
            ),
            DeclareLaunchArgument(
                "use_joint_state_publisher_gui",
                default_value="false",
                description="Run joint_state_publisher_gui for manual joint movement.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz2. Set false for headless validation.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("sawyer_description"), "config", "sawyer_ros2.rviz"]
                ),
                description="RViz2 config file path.",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_joint_state_publisher,
                            "' == 'true' and '",
                            use_joint_state_publisher_gui,
                            "' != 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                condition=IfCondition(
                    PythonExpression(
                        [
                            "'",
                            start_joint_state_publisher,
                            "' == 'true' and '",
                            use_joint_state_publisher_gui,
                            "' == 'true'",
                        ]
                    )
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
