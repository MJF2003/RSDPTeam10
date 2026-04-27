#!/usr/bin/python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('lslidar_catorgrapher')

    catorgrapher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'catorgrapher_calc.py')
        )
    )

    rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'catorgrapher_rviz.py')
        )
    )

    return LaunchDescription([
        catorgrapher_node,
        rviz_node,
    ])
