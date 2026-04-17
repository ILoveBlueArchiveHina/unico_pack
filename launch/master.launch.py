import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/uni-co-jetson/ros2_ws/src/unico_pack/launch/custom_bringup_v2.launch.py'))
    
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource('/home/uni-co-jetson/ros2_ws/src/kuo_pack/launch/mavros.launch.py'))

    

    return LaunchDescription([
        # USB camera node (landing cam)
        Node(
            package='unico_pack',
            executable='precision_landing_lifecycle',
            prefix = ['taskset -c 3']
        ),

        Node(
            package='unico_pack',
            executable='fast_lio_lifecycle_wrapper',
            prefix = ['taskset -c 4,5']
        )
        
    ])
    
