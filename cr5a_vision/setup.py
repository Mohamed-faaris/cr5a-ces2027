from setuptools import setup

package_name = "cr5a_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/vision.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Faaris",
    maintainer_email="mohamedfaaris.dev@gmail.com",
    description="CR5A RGB-D vision and calibrated 3D collision-box integration.",
    license="MIT",
    entry_points={"console_scripts": ["cr5a_vision_node = cr5a_vision.vision_node:main", "cr5a_apriltag_detector = cr5a_vision.apriltag_node:main"]},
)
