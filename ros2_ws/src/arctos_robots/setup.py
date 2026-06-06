import os
from glob import glob

from setuptools import find_packages, setup

package_name = "arctos_robots"


def _robot_data_files():
    """Install every robots/**/* file under share/<pkg>/robots, preserving tree."""
    entries = []
    for path in glob("robots/**/*", recursive=True):
        if os.path.isfile(path):
            entries.append((os.path.join("share", package_name, os.path.dirname(path)), [path]))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        *_robot_data_files(),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="3dberg",
    maintainer_email="tencho.tenchev@3dberg.com",
    description="Robot-type registry for the arctos-pi ROS2 stack.",
    license="TBD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "list_robots = arctos_robots.cli:list_robots",
        ],
    },
)
