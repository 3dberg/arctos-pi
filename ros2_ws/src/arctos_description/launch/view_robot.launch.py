"""Visualize the Arctos URDF in RViz with interactive joint sliders.

    ros2 launch arctos_description view_robot.launch.py

No hardware, no controllers — robot_state_publisher + joint_state_publisher_gui
only. Use this to sanity-check the geometry and joint limits (M1).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    description_pkg = FindPackageShare("arctos_description")

    gui = LaunchConfiguration("gui")

    robot_description = {
        "robot_description": Command(
            [
                FindExecutable(name="xacro"),
                " ",
                PathJoinSubstitution([description_pkg, "urdf", "arctos.urdf.xacro"]),
                " use_mock_components:=true",
            ]
        )
    }

    rviz_config = PathJoinSubstitution([description_pkg, "rviz", "view_robot.rviz"])

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch joint_state_publisher_gui with sliders.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(gui),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
