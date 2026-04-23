from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    realsense_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("realsense2_camera"),
            "/launch/rs_launch.py",
        ]
    )

    align_depth_enable = LaunchConfiguration("align_depth.enable")
    enable_sync = LaunchConfiguration("enable_sync")
    run_realsense_node = LaunchConfiguration("run_realsense_node")
    conf = LaunchConfiguration("conf")
    min_attr_conf = LaunchConfiguration("min_attr_conf")
    vote_window = LaunchConfiguration("vote_window")
    min_votes_to_output = LaunchConfiguration("min_votes_to_output")
    color_topic = LaunchConfiguration("color_topic")
    depth_topic = LaunchConfiguration("depth_topic")
    info_topic = LaunchConfiguration("info_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument("align_depth.enable", default_value="true"),
            DeclareLaunchArgument("enable_sync", default_value="true"),
            DeclareLaunchArgument("run_realsense_node", default_value="true"),
            DeclareLaunchArgument("conf", default_value="0.50"),
            DeclareLaunchArgument("min_attr_conf", default_value="0.20"),
            DeclareLaunchArgument("vote_window", default_value="10"),
            DeclareLaunchArgument("min_votes_to_output", default_value="5"),
            DeclareLaunchArgument(
                "color_topic", default_value="/camera/camera/color/image_raw"
            ),
            DeclareLaunchArgument(
                "depth_topic",
                default_value="/camera/camera/aligned_depth_to_color/image_raw",
            ),
            DeclareLaunchArgument(
                "info_topic", default_value="/camera/camera/color/camera_info"
            ),
            IncludeLaunchDescription(
                realsense_launch,
                condition=IfCondition(run_realsense_node),
                launch_arguments={
                    "align_depth.enable": align_depth_enable,
                    "enable_sync": enable_sync,
                }.items(),
            ),
            Node(
                package="rsdp_perception",
                executable="vision_node",
                name="vision_node",
                output="screen",
                parameters=[
                    {
                        "conf": conf,
                        "min_attr_conf": min_attr_conf,
                        "vote_window": vote_window,
                        "min_votes_to_output": min_votes_to_output,
                        "color_topic": color_topic,
                        "depth_topic": depth_topic,
                        "info_topic": info_topic,
                    }
                ],
            ),
        ]
    )
