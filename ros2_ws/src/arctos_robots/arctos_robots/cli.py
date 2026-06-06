"""Tiny CLI: `ros2 run arctos_robots list_robots`."""
from .registry import available_robots, load_robot


def list_robots() -> None:
    robots = available_robots()
    if not robots:
        print("no robots registered")
        return
    for name in robots:
        bundle = load_robot(name)
        print(f"{name}  (planning_group={bundle.planning_group})")
        print(f"    description: {bundle.description_xacro}")
        print(f"    srdf:        {bundle.srdf}")
