"""Pure-sim path: ros2_control controller_manager + mock_components.

    ros2 launch arctos_bringup mock_components.launch.py

No backend import, no CAN — mock_components/GenericSystem loops commands back as
state. Validates the URDF + ros2_control + MoveIt plumbing in isolation (M2).
"""
import xacro
from arctos_robots.registry import load_robot
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_args, **_kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    bundle = load_robot(robot_type)

    robot_description = xacro.process_file(
        bundle.description_xacro,
        mappings={"use_mock_components": "true"},
    ).toxml()

    return [
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{"robot_description": robot_description}, bundle.controllers_yaml],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster"],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["arctos_arm_controller"],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="arctos"),
            OpaqueFunction(function=_setup),
        ]
    )
