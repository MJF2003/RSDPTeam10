import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rsdp_perception"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "model_weights"),
            glob("model_weights/*.pt"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="laptop30",
    maintainer_email="jinweizhang2000@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "yolov5_realsense_node = rsdp_perception.yolov5_realsense_node:main",
            "vision_node = rsdp_perception.perception_typed_node:main",
        ],
    },
)
