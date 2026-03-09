import os

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("rover_gz_bringup")

    rover_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "rover_sim.launch.py")
        )
    )

    run_vision_stub_arg = DeclareLaunchArgument(
        "run_vision_stub",
        default_value="true",
        description="Run rover_sim_stubs vision_stub node",
    )
    run_navigation_stub_arg = DeclareLaunchArgument(
        "run_navigation_stub",
        default_value="true",
        description="Run rover_sim_stubs navigation_stub node",
    )
    run_smooth_observations_arg = DeclareLaunchArgument(
        "run_smooth_observations",
        default_value="true",
        description="Run rover_controller smooth_observations node",
    )
    run_rover_controller_arg = DeclareLaunchArgument(
        "run_rover_controller",
        default_value="true",
        description="Run rover_controller node",
    )

    vision_stub = Node(
        package="rover_sim_stubs",
        executable="vision_stub",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_vision_stub")),
    )

    navigation_stub = Node(
        package="rover_sim_stubs",
        executable="navigation_stub",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_navigation_stub")),
    )

    smooth_observations = Node(
        package="rover_controller",
        executable="smooth_observations",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_smooth_observations")),
    )

    rover_controller = Node(
        package="rover_controller",
        executable="rover_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_rover_controller")),
    )

    return LaunchDescription(
        [
            run_vision_stub_arg,
            run_navigation_stub_arg,
            run_smooth_observations_arg,
            run_rover_controller_arg,
            rover_sim,
            vision_stub,
            navigation_stub,
            smooth_observations,
            rover_controller,
        ]
    )
