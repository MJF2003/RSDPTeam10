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
    pkg_manipulation = get_package_share_directory("rover_manipulation")
    pkg_perception = get_package_share_directory("rsdp_perception")
    pkg_slam = get_package_share_directory("rover_slam")

    run_rover_manipulation = LaunchConfiguration("run_rover_manipulation")
    run_smooth_observations = LaunchConfiguration("run_smooth_observations")
    run_rover_controller = LaunchConfiguration("run_rover_controller")
    run_arm_joint_state_fallback = LaunchConfiguration("run_arm_joint_state_fallback")
    run_nav_debug_overlay = LaunchConfiguration("run_nav_debug_overlay")
    vision_process_every_n_frames = LaunchConfiguration("vision_process_every_n_frames")
    startup_observation_duration_sec = LaunchConfiguration(
        "startup_observation_duration_sec"
    )
    startup_observation_angular_z = LaunchConfiguration("startup_observation_angular_z")

    run_rover_manipulation_arg = DeclareLaunchArgument(
        "run_rover_manipulation",
        default_value="true",
        description="Run rover_manipulation arm_real.launch.py.",
    )
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
    run_nav_debug_overlay_arg = DeclareLaunchArgument(
        "run_nav_debug_overlay",
        default_value="true",
        description="Run rover_sim_stubs nav_debug_overlay node.",
    )
    vision_process_every_n_frames_arg = DeclareLaunchArgument(
        "vision_process_every_n_frames",
        default_value="1",
        description=(
            "Run YOLO once every N synchronized camera frames. "
            "Increase this to reduce vision CPU/GPU load."
        ),
    )
    startup_observation_duration_arg = DeclareLaunchArgument(
        "startup_observation_duration_sec",
        default_value="15.0",  # 2pi
        description="Startup camera sweep duration in seconds.",
    )
    startup_observation_angular_z_arg = DeclareLaunchArgument(
        "startup_observation_angular_z",
        default_value="0.5",
        description="Startup camera sweep angular velocity in rad/s.",
    )

    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "publish_description.py")
        ),
        launch_arguments={"robot_ns": ""}.items(),
    )

    rover_manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_manipulation, "launch", "arm_real.launch.py")
        ),
        condition=IfCondition(run_rover_manipulation),
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, "launch", "vision.launch.py")
        ),
        launch_arguments={
            "run_realsense_node": "true",
            "process_every_n_frames": vision_process_every_n_frames,
        }.items(),
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

    nav_debug_overlay = Node(
        package="rover_sim_stubs",
        executable="nav_debug_overlay",
        output="screen",
        parameters=[{"marker_frame": "map"}],
        condition=IfCondition(run_nav_debug_overlay),
    )

    return LaunchDescription(
        [
            run_rover_manipulation_arg,
            run_smooth_observations_arg,
            run_rover_controller_arg,
            run_arm_joint_state_fallback_arg,
            run_nav_debug_overlay_arg,
            vision_process_every_n_frames_arg,
            startup_observation_duration_arg,
            startup_observation_angular_z_arg,
            robot_description_launch,
            rover_manipulation_launch,
            vision_launch,
            slam_launch,
            smooth_observations,
            rover_controller,
            arm_joint_state_fallback,
            nav_debug_overlay,
        ]
    )
