"""Real-hardware path: robot_state_publisher + the arctos_bridge node.

    ros2 launch arctos_bringup arctos.launch.py can_backend:=mock
    ros2 launch arctos_bringup arctos.launch.py can_backend:=socketcan \\
        config_path:=/home/user/arctos-pi/config.yaml

The bridge reuses the tested Python Motion/CAN/MKS stack (MockBus on a dev box,
slcan/socketcan on the Pi) and serves the FollowJointTrajectory action MoveIt
drives. No controller_manager here — the bridge IS the controller.
"""
import xacro
from arctos_robots.registry import load_robot
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *_args, **_kwargs):
    robot_type = LaunchConfiguration("robot_type").perform(context)
    can_backend = LaunchConfiguration("can_backend").perform(context)
    config_path = LaunchConfiguration("config_path").perform(context)

    bundle = load_robot(robot_type)
    robot_description = xacro.process_file(
        bundle.description_xacro,
        mappings={"use_mock_components": "true"},
    ).toxml()

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="arctos_bridge",
            executable="bridge_node",
            output="screen",
            parameters=[
                {
                    "config_path": config_path,
                    "can_backend": can_backend,
                    "controller_name": "arctos_arm_controller",
                }
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="arctos"),
            DeclareLaunchArgument(
                "can_backend",
                default_value="mock",
                description="mock | dry_run | slcan | socketcan (overrides config).",
            ),
            DeclareLaunchArgument(
                "config_path",
                default_value="",
                description="Path to arctos-pi config.yaml (empty = built-in defaults).",
            ),
            OpaqueFunction(function=_setup),
        ]
    )
