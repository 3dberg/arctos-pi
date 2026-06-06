"""Start MoveIt2 move_group for the Arctos arm.

    ros2 launch arctos_moveit_config move_group.launch.py

Execution is routed to the FollowJointTrajectory action served by either the
ros2_control joint_trajectory_controller (mock_components.launch.py) or the
arctos_bridge node (arctos.launch.py).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _moveit_configs():
    desc = get_package_share_directory("arctos_description")
    return (
        MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config")
        .robot_description(
            file_path=os.path.join(desc, "urdf", "arctos.urdf.xacro"),
            mappings={"use_mock_components": "true"},
        )
        .robot_description_semantic(file_path="config/arctos.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )


def generate_launch_description():
    moveit_config = _moveit_configs()
    return LaunchDescription(
        [
            # Accepted for forward-compat with the registry; only 'arctos' today.
            DeclareLaunchArgument("robot_type", default_value="arctos"),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=[moveit_config.to_dict()],
            ),
        ]
    )
