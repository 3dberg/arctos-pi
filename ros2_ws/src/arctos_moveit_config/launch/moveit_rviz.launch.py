"""RViz with the MoveIt MotionPlanning panel for the Arctos arm.

    ros2 launch arctos_moveit_config moveit_rviz.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    desc = get_package_share_directory("arctos_description")
    moveit_share = get_package_share_directory("arctos_moveit_config")

    moveit_config = (
        MoveItConfigsBuilder("arctos", package_name="arctos_moveit_config")
        .robot_description(
            file_path=os.path.join(desc, "urdf", "arctos.urdf.xacro"),
            mappings={"use_mock_components": "true"},
        )
        .robot_description_semantic(file_path="config/arctos.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    rviz_config = os.path.join(moveit_share, "config", "moveit.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="arctos"),
            Node(
                package="rviz2",
                executable="rviz2",
                name="moveit_rviz",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )
