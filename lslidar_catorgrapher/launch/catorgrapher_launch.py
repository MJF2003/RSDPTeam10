from launch import LaunchDescription  # 导入LaunchDescription类，用于定义launch文件的描述
from ament_index_python.packages import get_package_share_directory  # 用于获取ROS 2包的共享目录
from launch_ros.actions import Node, LifecycleNode  # 导入Node和LifecycleNode类，用于启动ROS 2节点
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription  # 导入声明参数、定时器动作和包含其他launch文件的类
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution  # 导入LaunchConfiguration和PathJoinSubstitution类，用于参数替换
from launch.launch_description_sources import PythonLaunchDescriptionSource  # 导入PythonLaunchDescriptionSource类，用于加载其他launch文件
import os
def generate_launch_description():
    
    catorgrapher_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("lslidar_catorgrapher"),"/launch","/catorgrapher_calc.py"]),
    )
    
#    lslidar_node = IncludeLaunchDescription(
#        PythonLaunchDescriptionSource([get_package_share_directory("lslidar_catorgrapher"), "/launch", "/catorgrapher_laser.py"]),
#    )
    
    lslidar_rviz_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("lslidar_catorgrapher"), "/launch", "/catorgrapher_rviz.py"]),
    )

    return LaunchDescription([
#        lslidar_node,
        TimerAction(
            period=6.0,  
            actions=[catorgrapher_node]
        ),
        TimerAction(
            period=9.0,  
            actions=[lslidar_rviz_node]
        )
    ])

