import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory("rover_gz_bringup")
    pkg_perception = get_package_share_directory("rsdp_perception")
    pkg_slam = get_package_share_directory("rover_slam")
    robot_ns = LaunchConfiguration("robot_ns")

    def namespaced_topic(topic_suffix):
        return PythonExpression(
            [
                "'/' + '",
                robot_ns,
                "' + '",
                topic_suffix,
                "' if '",
                robot_ns,
                "' else '",
                topic_suffix,
                "'",
            ]
        )

    rover_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, "launch", "rover_sim.launch.py")
        ),
        launch_arguments={
            "robot_ns": robot_ns,
            "headless": LaunchConfiguration("headless"),
        }.items(),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, "launch", "slam_launch.py")
        ),
        launch_arguments={
            "launch_rviz": "false",
            "launch_rplidar": "false",
        }.items(),
        condition=IfCondition(LaunchConfiguration("run_slam_node")),
    )

    robot_ns_arg = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace for the Gazebo sim and fallback joint publishers.",
    )
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Launch Gazebo without the GUI while keeping sensor rendering available.",
    )

    run_vision_stub_arg = DeclareLaunchArgument(
        "run_vision_stub",
        default_value="true",
        description="Run rover_sim_stubs vision_stub node",
    )
    run_vision_true_arg = DeclareLaunchArgument(
        "run_vision_true",
        default_value="false",
        description="Run rsdp_perception vision.launch.py against simulated camera topics",
    )
    run_navigation_stub_arg = DeclareLaunchArgument(
        "run_navigation_stub",
        default_value="true",
        description="Run rover_sim_stubs navigation_stub node",
    )
    run_manipulation_stub_arg = DeclareLaunchArgument(
        "run_manipulation_stub",
        default_value="true",
        description="Run rover_sim_stubs manipulation_stub node",
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
    run_nav_debug_overlay_arg = DeclareLaunchArgument(
        "run_nav_debug_overlay",
        default_value="false",
        description="Run rover_sim_stubs nav_debug_overlay node",
    )
    run_arm_joint_state_fallback_arg = DeclareLaunchArgument(
        "run_arm_joint_state_fallback",
        default_value="true",
        description="Publish fallback arm joint states for sim when no real arm stack is running.",
    )

    run_slam_node_arg = DeclareLaunchArgument(
        "run_slam_node",
        default_value="true",
        description="run rover_slam to convert mocked /scan to /map.",
    )

    vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_perception, "launch", "vision.launch.py")
        ),
        launch_arguments={
            "run_realsense_node": "false",
            "color_topic": namespaced_topic("/depth_camera/image"),
            "depth_topic": namespaced_topic("/depth_camera/depth_image"),
            "info_topic": namespaced_topic("/depth_camera/camera_info"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("run_vision_true")),
    )

    vision_stub = Node(
        package="rover_sim_stubs",
        executable="vision_stub",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "'",
                    LaunchConfiguration("run_vision_stub"),
                    "'.lower() in ('true', '1') and '",
                    LaunchConfiguration("run_vision_true"),
                    "'.lower() not in ('true', '1')",
                ]
            )
        ),
    )

    navigation_stub = Node(
        package="rover_sim_stubs",
        executable="navigation_stub",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_navigation_stub")),
    )

    navigation_stub_warning = LogInfo(
        msg=(
            "launch_sim.py: run_navigation_stub:=true exposes /navigate_to_pos. "
            "Set run_navigation_stub:=false when launching navigation_2 to avoid "
            "competing navigation providers."
        ),
        condition=IfCondition(LaunchConfiguration("run_navigation_stub")),
    )

    manipulation_stub = Node(
        package="rover_sim_stubs",
        executable="manipulation_stub",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_manipulation_stub")),
    )

    smooth_observations = Node(
        package="rover_controller",
        executable="smooth_observations",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("run_smooth_observations")),
    )

    rover_controller = Node(
        package="rover_controller",
        executable="rover_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_rover_controller")),
    )

    nav_debug_overlay = Node(
        package="rover_sim_stubs",
        executable="nav_debug_overlay",
        output="screen",
        condition=IfCondition(LaunchConfiguration("run_nav_debug_overlay")),
    )

    arm_joint_state_fallback = Node(
        namespace=robot_ns,
        package="rover_controller",
        executable="sim_arm_joint_state_publisher",
        name="sim_arm_joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("run_arm_joint_state_fallback")),
    )

    return LaunchDescription(
        [
            robot_ns_arg,
            headless_arg,
            run_vision_stub_arg,
            run_vision_true_arg,
            run_navigation_stub_arg,
            run_manipulation_stub_arg,
            run_smooth_observations_arg,
            run_rover_controller_arg,
            run_nav_debug_overlay_arg,
            run_arm_joint_state_fallback_arg,
            run_slam_node_arg,
            rover_sim,
            slam_launch,
            vision_launch,
            vision_stub,
            navigation_stub_warning,
            navigation_stub,
            manipulation_stub,
            smooth_observations,
            rover_controller,
            nav_debug_overlay,
            arm_joint_state_fallback,
        ]
    )
