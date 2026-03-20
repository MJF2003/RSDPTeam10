import os
from glob import glob

from setuptools import find_packages, setup

package_name = "rsdp_perception"


package_name = "rsdp_perception"


def get_data_files(directory):
    paths = []
    for path, _, filenames in os.walk(directory):
        for filename in filenames:
            paths.append(
                (
                    os.path.join("share", package_name, path),
                    [os.path.join(path, filename)],
                )
            )
    return paths


data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
] + get_data_files("model_weights")


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
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
