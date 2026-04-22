#!/usr/bin/python3
import os

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetParameter
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

    wander_node = Node(
        package='leo_explore',
        executable='wander',
        name='leo_wander',
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',

                'forward_speed': 0.50,
                'slow_speed': 0.18,
                'commit_forward_speed': 0.18,
                'min_forward_speed': 0.14,

                'turn_speed': 0.58,
                'emergency_turn_speed': 0.85,

                'safe_dist': 0.30,
                'slow_dist': 0.50,
                'emergency_dist': 0.16,
                'side_safe_dist': 0.24,

                'release_front_dist': 0.42,
                'release_side_dist': 0.30,

                'commit_min_time': 1.80,
                'commit_max_time': 5.00,
                'direction_deadband': 0.12,

                'robot_stop_timeout': 0.50,
                'angular_sign': -1.0,
            }
        ]
    )

    return LaunchDescription([
        GroupAction([
            SetParameter(name='use_sim_time', value=True),
            TimerAction(period=2.0, actions=[cartographer_calc_launch]),
            TimerAction(period=4.0, actions=[rviz_launch]),
            TimerAction(period=8.0, actions=[wander_node]),
        ])
    ])
