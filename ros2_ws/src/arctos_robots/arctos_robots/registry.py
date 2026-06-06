"""Robot-type registry: map a robot_type to its description / MoveIt bundle.

A robot type is declared by a manifest at
``<share>/arctos_robots/robots/<robot_type>/robot.yaml`` whose entries are
``package:relative/path`` references. ``load_robot()`` resolves those into
absolute paths (via the ament index) so launch files can stay robot-agnostic:
they take a ``robot_type`` argument and ask the registry instead of hardcoding
package paths.

Adding a new arm = drop a new ``robots/<type>/robot.yaml`` plus its
description / moveit_config packages. No launch-file edits.
"""
from __future__ import annotations

import os
from dataclasses import dataclass, field

import yaml
from ament_index_python.packages import get_package_share_directory

DEFAULT_ROBOT_TYPE = "arctos"


@dataclass
class RobotBundle:
    """Resolved (absolute-path) bundle for one robot type."""
    robot_type: str
    description_xacro: str
    srdf: str
    controllers_yaml: str
    moveit_controllers: str
    kinematics: str
    joint_limits: str
    ompl_planning: str
    planning_group: str
    raw: dict = field(default_factory=dict)


def _resolve(ref: str) -> str:
    """Resolve a 'package:relative/path' reference to an absolute share path."""
    if ":" not in ref:
        raise ValueError(
            f"manifest reference {ref!r} must be 'package:relative/path'"
        )
    pkg, rel = ref.split(":", 1)
    return os.path.join(get_package_share_directory(pkg), rel)


def robots_root() -> str:
    return os.path.join(get_package_share_directory("arctos_robots"), "robots")


def available_robots() -> list[str]:
    """List robot types that have a robot.yaml manifest."""
    root = robots_root()
    if not os.path.isdir(root):
        return []
    return sorted(
        name
        for name in os.listdir(root)
        if os.path.isfile(os.path.join(root, name, "robot.yaml"))
    )


def load_robot(robot_type: str = DEFAULT_ROBOT_TYPE) -> RobotBundle:
    manifest = os.path.join(robots_root(), robot_type, "robot.yaml")
    if not os.path.isfile(manifest):
        raise KeyError(
            f"unknown robot_type {robot_type!r}; available: {available_robots()}"
        )
    with open(manifest) as fh:
        data = yaml.safe_load(fh) or {}

    return RobotBundle(
        robot_type=data.get("robot_type", robot_type),
        description_xacro=_resolve(data["description_xacro"]),
        srdf=_resolve(data["srdf"]),
        controllers_yaml=_resolve(data["controllers_yaml"]),
        moveit_controllers=_resolve(data["moveit_controllers"]),
        kinematics=_resolve(data["kinematics"]),
        joint_limits=_resolve(data["joint_limits"]),
        ompl_planning=_resolve(data["ompl_planning"]),
        planning_group=data.get("planning_group", "arm"),
        raw=data,
    )
