#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('lslidar_catorgrapher')
    config_dir = os.path.join(pkg_share, 'config')

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'catorgrapher_config.lua'
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )

    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0'
        ],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        cartographer_node,
        occupancy_grid_node,
    ])
