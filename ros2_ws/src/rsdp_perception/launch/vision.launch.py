from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    realsense_launch = PythonLaunchDescriptionSource(
        [
            get_package_share_directory("realsense2_camera"),
            "/launch/rs_launch.py",
        ]
    )

    align_depth_enable = LaunchConfiguration("align_depth.enable")
    enable_sync = LaunchConfiguration("enable_sync")
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
            DeclareLaunchArgument("conf", default_value="0.25"),
            DeclareLaunchArgument("min_attr_conf", default_value="0.20"),
            DeclareLaunchArgument("vote_window", default_value="10"),
            DeclareLaunchArgument("min_votes_to_output", default_value="3"),
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
