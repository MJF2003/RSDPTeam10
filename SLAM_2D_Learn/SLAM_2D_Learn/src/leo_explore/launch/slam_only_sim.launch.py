#!/usr/bin/python3
import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    cartographer_calc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lslidar_catorgrapher'),
                'launch',
                'catorgrapher_calc.py'
            )
        )
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lslidar_catorgrapher'),
                'launch',
                'catorgrapher_rviz.py'
            )
        )
    )

    return LaunchDescription([
        GroupAction([
            SetParameter(name='use_sim_time', value=True),
            TimerAction(period=2.0, actions=[cartographer_calc_launch]),
            TimerAction(period=4.0, actions=[rviz_launch]),
        ])
    ])
