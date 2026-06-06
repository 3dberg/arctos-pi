"""End-to-end dev demo: real-stack bridge + MoveIt2 + RViz.

    ros2 launch arctos_bringup demo.launch.py            # MockBus
    ros2 launch arctos_bringup demo.launch.py can_backend:=socketcan \\
        config_path:=/home/user/arctos-pi/config.yaml

In RViz MotionPlanning: set a goal -> Plan -> Execute. The trajectory flows
move_group -> arctos_bridge -> Motion -> CanBus (degrees<->pulses).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup = get_package_share_directory("arctos_bringup")
    moveit = get_package_share_directory("arctos_moveit_config")

    robot_type = LaunchConfiguration("robot_type")
    can_backend = LaunchConfiguration("can_backend")
    config_path = LaunchConfiguration("config_path")

    arctos = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup, "launch", "arctos.launch.py")),
        launch_arguments={
            "robot_type": robot_type,
            "can_backend": can_backend,
            "config_path": config_path,
        }.items(),
    )

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit, "launch", "move_group.launch.py")),
        launch_arguments={"robot_type": robot_type}.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(moveit, "launch", "moveit_rviz.launch.py")),
        launch_arguments={"robot_type": robot_type}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_type", default_value="arctos"),
            DeclareLaunchArgument("can_backend", default_value="mock"),
            DeclareLaunchArgument("config_path", default_value=""),
            arctos,
            move_group,
            rviz,
        ]
    )
