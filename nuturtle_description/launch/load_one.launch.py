from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    PathJoinSubstitution,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.conditions import IfCondition
from launch.actions import Shutdown


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value=["true"],
                description="for launching rviz or not",
            ),
            DeclareLaunchArgument(
                "use_jsp",
                default_value=["true"],
                description="whether the jsp is published or not",
            ),
            DeclareLaunchArgument(
                "color",
                default_value=["purple"],
                description="color of the robot",
                choices=["purple", "red", "green", "blue"],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": Command(
                            [
                                ExecutableInPackage("xacro", "xacro"),
                                " ",
                                PathJoinSubstitution(
                                    [
                                        FindPackageShare("nuturtle_description"),
                                        "urdf/turtlebot3_burger.urdf.xacro",
                                    ]
                                ),
                                " ",
                                "color:=",
                                LaunchConfiguration("color"),
                            ]
                        ),
                        "frame_prefix": [LaunchConfiguration("color"), "/"],
                    }
                ],
                namespace=LaunchConfiguration("color"),
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("use_jsp"), "' == 'true' "]
                    )
                ),
                namespace=LaunchConfiguration("color"),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration("use_rviz"), "' == 'true' "]
                    )
                ),
                arguments=[
                    "-d",
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nuturtle_description"),
                            "config/basic_purple.rviz",
                        ]
                    ),
                    "-f",
                    [LaunchConfiguration("color"), "/base_footprint"],
                ],
                namespace=LaunchConfiguration("color"),
                on_exit=Shutdown(),
            ),
        ]
    )
