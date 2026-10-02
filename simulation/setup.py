# Author: Luca Obwegs
from glob import glob
from setuptools import find_packages, setup

package_name = "balancing_robot_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
        ("share/" + package_name + "/models/balancing_robot", glob("models/balancing_robot/*")),
        ("share/" + package_name + "/worlds", glob("worlds/*")),
    ],
    install_requires=["setuptools", "numpy", "scipy"],
    zip_safe=True,
    maintainer="Luca Obwegs",
    maintainer_email="luca.obwegs@hilti.com",
    description="Gazebo Harmonic balancing robot simulation and controllers",
    license="GPL-3.0-only",
    entry_points={
        "console_scripts": [
            "balance_controller = balancing_robot_sim.ros_controller:main",
            "generate_model = balancing_robot_sim.generate_sdf:main",
        ],
    },
)
