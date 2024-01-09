from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution, \
                                    LaunchConfiguration, PythonExpression
from launch_ros.substitutions import ExecutableInPackage, FindPackageShare
from launch.conditions import IfCondition
from launch.actions import Shutdown

def generate_launch_description():
    return LaunchDescription([

        DeclareLaunchArgument(
            "use_rviz",
            default_value=["true"],
            description="for launching rviz or not"
        ),

        DeclareLaunchArgument(
            "use_jsp",
            default_value=["true"],
            description="whether the jsp is published or not"
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            # arguments=[LaunchConfiguration("use_rviz")],
            # condition=IfCondition(PythonExpression(["'",
            #                                         LaunchConfiguration('use_rviz'),"' == \'true\' "])),
            parameters=[
                        {"robot_description":
                            Command([ExecutableInPackage("xacro", "xacro"), " ",
                                    PathJoinSubstitution(
                                        [FindPackageShare("nuturtle_description"), "urdf/turtlebot3_burger.urdf.xacro"])])}
                        ]
            ),

        Node(
            package="joint_state_publisher",
            executable="joint_state_publisher",
            condition=IfCondition(PythonExpression(["'",
                                                    LaunchConfiguration('use_jsp'),
                                                    "' == \'true\' "])),
            ),

        # Node(
        #     package="joint_state_publisher_gui",
        #     executable="joint_state_publisher_gui",
        #     arguments=[LaunchConfiguration("use_jsp")],
        #     condition=IfCondition(PythonExpression(["'",
        #                                             LaunchConfiguration('use_jsp'),
        #                                             "' == \'gui\' "])),

        # ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            condition=IfCondition(PythonExpression(["'",
                                                    LaunchConfiguration('use_rviz'),"' == \'true\' "])),
            arguments=["-d", PathJoinSubstitution(
                [FindPackageShare("nuturtle_description"),
                 "config/basic_purple.rviz"])],
            on_exit=Shutdown()
            ),
    ])
