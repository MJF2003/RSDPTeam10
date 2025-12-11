#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
import os

def generate_launch_description():

    # 获取包路径
    pkg_share = get_package_share_directory('lslidar_catorgrapher')
    
    # 配置文件路径
    config_dir = os.path.join(pkg_share, 'config')
    urdf_file = os.path.join(config_dir, 'catorgrapher_urdf.urdf')
    
    # 读取URDF文件
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    # cartographer_node 节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        emulate_tty=True,
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', 'catorgrapher_config.lua'
        ],
        parameters=[{
            'use_sim_time': False
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom')
        ]
    )

    # cartographer_occupancy_grid_node 节点
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
            'resolution': 0.05
        }]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])