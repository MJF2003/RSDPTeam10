import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rover_manipulation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laptop20",
    maintainer_email="alexinch96@yahoo.com",
    description="Real-arm manipulation wrappers for the Team 10 rover.",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "auto_pick = rover_manipulation.auto_pick:main",
            "tcp_bridge = rover_manipulation.tcp_bridge:main",
        ],
    },
)
