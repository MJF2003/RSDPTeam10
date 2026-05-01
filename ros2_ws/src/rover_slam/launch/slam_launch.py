#!/usr/bin/python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_slam")
    config_dir = os.path.join(pkg_share, "config")
    rviz_config = os.path.join(config_dir, "catorgrapher_rviz.rviz")
    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rplidar = LaunchConfiguration("launch_rplidar")
    use_sim_time = LaunchConfiguration("use_sim_time")

    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="false",
        description="Launch RViz alongside the SLAM stack.",
    )

    launch_rplidar_arg = DeclareLaunchArgument(
        "launch_rplidar",
        default_value="false",
        description="Launch the RPLIDAR driver alongside the SLAM stack.",
    )
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Gazebo /clock when running SLAM in simulation. Set to false for hardware.",
    )

    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_to_laser_tf",
        output="screen",
        arguments=["0.0700", "0", "0.0380", "0", "0", "0", "base_link", "laser"],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        arguments=[
            "-configuration_directory",
            config_dir,
            "-configuration_basename",
            "catorgrapher_config.lua",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("scan", "/scan"),
        ],
    )

    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="cartographer_occupancy_grid_node",
        output="screen",
        arguments=[
            "-resolution",
            "0.05",
            "-publish_period_sec",
            "1.0",
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rplidar_ros"),
                "launch",
                "rplidar_a2m12_launch.py",
            )
        ),
        launch_arguments={
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": "256000",
            "frame_id": "laser",
            "inverted": "false",
            "angle_compensate": "true",
            "scan_mode": "Sensitivity",
        }.items(),
        condition=IfCondition(launch_rplidar),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(launch_rviz),
    )

    return LaunchDescription(
        [
            launch_rviz_arg,
            launch_rplidar_arg,
            use_sim_time_arg,
            static_tf_laser,
            cartographer_node,
            occupancy_grid_node,
            rplidar_launch,
            rviz_node,
        ]
    )
