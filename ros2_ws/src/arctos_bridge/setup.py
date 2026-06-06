import os
from glob import glob

from setuptools import find_packages, setup

package_name = "arctos_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="3dberg",
    maintainer_email="tencho.tenchev@3dberg.com",
    description="rclpy bridge from MoveIt2 to the arctos-pi Python control stack.",
    license="TBD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = arctos_bridge.bridge_node:main",
        ],
    },
)
