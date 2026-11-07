from setuptools import setup
from glob import glob
import os

package_name = "cr5a_ros_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Faaris",
    maintainer_email="mohamedfaaris.dev@gmail.com",
    description="Headless REST and WebSocket gateway for the Dobot CR5A ROS 2 stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "cr5a_ros_bridge = cr5a_ros_bridge.main:main",
        ],
    },
)
