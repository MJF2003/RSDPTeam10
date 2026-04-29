import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_perception = get_package_share_directory("rsdp_perception")
    pkg_slam = get_package_share_directory("rover_slam")

    run_smooth_observations = LaunchConfiguration("run_smooth_observations")
    run_rover_controller = LaunchConfiguration("run_rover_controller")
    run_arm_joint_state_fallback = LaunchConfiguration("run_arm_joint_state_fallback")
    startup_observation_duration_sec = LaunchConfiguration(
        "startup_observation_duration_sec"
    )
    startup_observation_angular_z = LaunchConfiguration("startup_observation_angular_z")

    run_smooth_observations_arg = DeclareLaunchArgument(
        "run_smooth_observations",
        default_value="true",
        description="Run rover_controller smooth_observations node.",
    )
    run_rover_controller_arg = DeclareLaunchArgument(
        "run_rover_controller",
        default_value="true",
        description="Run rover_controller node.",
    )
    run_arm_joint_state_fallback_arg = DeclareLaunchArgument(
        "run_arm_joint_state_fallback",
        default_value="false",
        description=(
            "Publish fallback arm joint states when the real arm stack is not running."
        ),
    )
    startup_observation_duration_arg = DeclareLaunchArgument(
        "startup_observation_duration_sec",
        default_value="6.28",  # 2pi
        description="Startup camera sweep duration in seconds.",
    )
    startup_observation_angular_z_arg = DeclareLaunchArgument(
        "startup_observation_angular_z",
        default_value="1.0",
        description="Startup camera sweep angular velocity in rad/s.",
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, "launch", "vision.launch.py")
        ),
        launch_arguments={"run_realsense_node": "true"}.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, "launch", "slam_launch.py")
        ),
        launch_arguments={
            "launch_rviz": "false",
            "launch_rplidar": "true",
            "use_sim_time": "false",
        }.items(),
    )

    smooth_observations = Node(
        package="rover_controller",
        executable="smooth_observations",
        output="screen",
        condition=IfCondition(run_smooth_observations),
    )

    rover_controller = Node(
        package="rover_controller",
        executable="rover_controller",
        output="screen",
        parameters=[
            {
                "startup_observation_duration_sec": (startup_observation_duration_sec),
                "startup_observation_angular_z": startup_observation_angular_z,
            }
        ],
        condition=IfCondition(run_rover_controller),
    )

    arm_joint_state_fallback = Node(
        package="rover_controller",
        executable="sim_arm_joint_state_publisher",
        name="sim_arm_joint_state_publisher",
        output="screen",
        condition=IfCondition(run_arm_joint_state_fallback),
    )

    return LaunchDescription(
        [
            run_smooth_observations_arg,
            run_rover_controller_arg,
            run_arm_joint_state_fallback_arg,
            startup_observation_duration_arg,
            startup_observation_angular_z_arg,
            vision_launch,
            slam_launch,
            smooth_observations,
            rover_controller,
            arm_joint_state_fallback,
        ]
    )
