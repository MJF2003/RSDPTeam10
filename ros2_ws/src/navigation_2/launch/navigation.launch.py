import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    nav_dir = get_package_share_directory('navigation_2')

    # Default configuration file path
    default_params_file = os.path.join(nav_dir, 'config', 'neo_robot.yaml')
    default_nav_to_pose_bt_xml = os.path.join(
        nav_dir,
        'config',
        'behavior_trees',
        'navigate_to_pose_w_replanning_and_recovery.xml',
    )
    default_nav_through_poses_bt_xml = os.path.join(
        nav_dir,
        'config',
        'behavior_trees',
        'navigate_through_poses_w_replanning_and_recovery.xml',
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    run_explore_server = LaunchConfiguration('run_explore_server')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites={
                'bt_navigator.ros__parameters.use_sim_time': use_sim_time,
                'bt_navigator.ros__parameters.default_nav_to_pose_bt_xml': (
                    default_nav_to_pose_bt_xml
                ),
                'bt_navigator.ros__parameters.default_nav_through_poses_bt_xml': (
                    default_nav_through_poses_bt_xml
                ),
                'controller_server.ros__parameters.use_sim_time': use_sim_time,
                'local_costmap.local_costmap.ros__parameters.use_sim_time': (
                    use_sim_time
                ),
                'global_costmap.global_costmap.ros__parameters.use_sim_time': (
                    use_sim_time
                ),
                'planner_server.ros__parameters.use_sim_time': use_sim_time,
                'behavior_server.ros__parameters.use_sim_time': use_sim_time,
                'waypoint_follower.ros__parameters.use_sim_time': use_sim_time,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    # List of Lifecycle Milestones
    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),

        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('run_explore_server', default_value='true'),

        # 1. Controller Server
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[configured_params]),

        # 2. Planner Server
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[configured_params]),

        # 3. Behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[configured_params]),

        # 4. BT Navigator
        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[configured_params]),

        # 5. Route Tracker
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[configured_params]),

        # 6. Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'autostart': autostart,
                         'node_names': lifecycle_nodes}]),

        # 7. Navigation Service Node
        Node(
            package='navigation_2',
            executable='navigation_service_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        # 8. Frontier Exploration Action Server
        Node(
            package='leo_explore',
            executable='explore_action_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(run_explore_server)),
    ])
