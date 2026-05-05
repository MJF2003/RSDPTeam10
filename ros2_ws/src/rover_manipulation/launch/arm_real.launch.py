import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_mycobot_moveit = get_package_share_directory("mycobot_280_moveit2")

    pi_host = LaunchConfiguration("pi_host")
    pi_port = LaunchConfiguration("pi_port")
    run_moveit_demo = LaunchConfiguration("run_moveit_demo")
    run_tcp_bridge = LaunchConfiguration("run_tcp_bridge")
    run_auto_pick = LaunchConfiguration("run_auto_pick")

    pi_host_arg = DeclareLaunchArgument(
        "pi_host",
        default_value="10.3.14.59",
        description="Arm-side TCP server host.",
    )
    pi_port_arg = DeclareLaunchArgument(
        "pi_port",
        default_value="9999",
        description="Arm-side TCP server port.",
    )
    run_moveit_demo_arg = DeclareLaunchArgument(
        "run_moveit_demo",
        default_value="true",
        description="Run mycobot_280_moveit2 demo launch.",
    )
    run_tcp_bridge_arg = DeclareLaunchArgument(
        "run_tcp_bridge",
        default_value="true",
        description="Run the NUC-to-arm TCP bridge.",
    )
    run_auto_pick_arg = DeclareLaunchArgument(
        "run_auto_pick",
        default_value="true",
        description="Run the ManipulateBlock action server.",
    )

    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_mycobot_moveit, "launch", "demo.launch.py")
        ),
        condition=IfCondition(run_moveit_demo),
    )

    tcp_bridge = Node(
        package="rover_manipulation",
        executable="tcp_bridge",
        name="smooth_tcp_bridge",
        output="screen",
        parameters=[
            {
                "pi_host": pi_host,
                "pi_port": ParameterValue(pi_port, value_type=int),
            }
        ],
        condition=IfCondition(run_tcp_bridge),
    )

    auto_pick = Node(
        package="rover_manipulation",
        executable="auto_pick",
        name="auto_pick_node",
        output="screen",
        condition=IfCondition(run_auto_pick),
    )

    return LaunchDescription(
        [
            pi_host_arg,
            pi_port_arg,
            run_moveit_demo_arg,
            run_tcp_bridge_arg,
            run_auto_pick_arg,
            moveit_demo_launch,
            tcp_bridge,
            auto_pick,
        ]
    )
