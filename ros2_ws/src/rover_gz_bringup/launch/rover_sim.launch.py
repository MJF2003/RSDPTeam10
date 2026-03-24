# Copyright 2023 Fictionlab sp. z o.o.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

# =================================
# Customise LeoRover launch file for sim launch
# Adds all necessary resource
# Launches world and triggers rover spawn
# =================================

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Setup project paths
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_project_gazebo = get_package_share_directory("rover_gz_bringup")
    pkg_project_worlds = get_package_share_directory("leo_gz_worlds")

    # Added to permit the search of other packages for robot description resources
    mycobot_path = get_package_share_directory('mycobot_description')
    rover_path = get_package_share_directory('rover_description')

    mycobot_base_path = os.path.dirname(mycobot_path)
    rover_base_path = os.path.dirname(rover_path)

    gazebo_model_paths = mycobot_base_path + os.pathsep + rover_base_path


    sim_world = DeclareLaunchArgument(
        "sim_world",
        default_value=os.path.join(
            pkg_project_worlds, "worlds", "pick_place_arena.sdf"
        ),
        description="Path to the Gazebo world file",
    )

    robot_ns = DeclareLaunchArgument(
        "robot_ns",
        default_value="",
        description="Robot namespace",
    )

    # Setup to launch the simulator and Gazebo world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={"gz_args": LaunchConfiguration("sim_world")}.items(),
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_project_gazebo, "launch", "spawn_robot.launch.py")
        ),
        launch_arguments={"robot_ns": LaunchConfiguration("robot_ns")}.items(),
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    topic_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # Block, bin, platform positions for the vision stub module
            "/model/block_red_1/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/block_blue_1/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/block_yellow_1/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/bin_red_1/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/bin_blue_1/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/bin_yellow_1/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/platform_red/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/platform_blue/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
            "/model/platform_yellow/pose@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V",
        ],
        parameters=[
            {
                "qos_overrides./tf_static.publisher.durability": "transient_local",
            }
        ],
        output="screen",
    )



    return LaunchDescription(
        # Add the resource location of Gazebo models to the runtime environment
        [   
            AppendEnvironmentVariable(          
            name='GZ_SIM_RESOURCE_PATH',
            value=gazebo_model_paths
        ), 
            sim_world,
            robot_ns,
            gz_sim,
            spawn_robot,
            topic_bridge,
        ]
    )
