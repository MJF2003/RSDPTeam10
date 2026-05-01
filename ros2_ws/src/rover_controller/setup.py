import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rover_controller"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools", "numpy"],
    zip_safe=True,
    maintainer="laptop20",
    maintainer_email="alexinch96@yahoo.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "rover_controller = rover_controller.rover_controller:main",
            "mission_console = rover_controller.mission_console:main",
            "smooth_observations = rover_controller.handle_observations:main",
            "sim_arm_joint_state_publisher = rover_controller.sim_arm_joint_state_publisher:main",
        ],
    },
)
