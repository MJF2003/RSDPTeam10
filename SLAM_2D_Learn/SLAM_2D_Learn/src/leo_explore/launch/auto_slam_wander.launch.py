#!/usr/bin/python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rplidar_ros'),
                'launch',
                'rplidar_a2m12_launch.py'
            )
        ),
        launch_arguments={
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': '256000',
            'frame_id': 'laser',
            'inverted': 'false',
            'angle_compensate': 'true',
            'scan_mode': 'Sensitivity',
        }.items()
    )

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
                'scan_topic': '/scan',
                'cmd_vel_topic': '/cmd_vel',
                'forward_speed': 0.09,
                'min_speed': 0.035,
                'max_turn_speed': 0.85,
                'safe_dist': 0.32,
                'slow_dist': 0.55,
                'emergency_dist': 0.20,
                'robot_stop_timeout': 0.50,
                'angular_sign': -1.0,
                'local_angle_range_deg': 100.0,
                'goal_bias_deg': 18.0,
                'gap_window_deg': 12.0,
                'target_smooth_alpha': 0.68,
                'emergency_turn_hold': 0.45,
                'max_considered_range': 3.5,
                'commit_turn_hold': 1.00,
                'commit_angle_deg': 24.0,
                'commit_score_bias': 0.35,
                'front_release_margin': 0.10,
            }
        ]
    )

    return LaunchDescription([
        rplidar_launch,
        TimerAction(period=2.0, actions=[cartographer_calc_launch]),
        TimerAction(period=4.0, actions=[rviz_launch]),
        TimerAction(period=8.0, actions=[wander_node]),
    ])
