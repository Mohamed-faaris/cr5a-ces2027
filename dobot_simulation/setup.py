from setuptools import setup

package_name = "dobot_simulation"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mohamed Faaris",
    maintainer_email="mohamedfaaris.dev@gmail.com",
    description="Simulation-only Dobot CR5A telemetry and dashboard services.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "dobot_simulation = dobot_simulation.simulator:main",
        ],
    },
)
