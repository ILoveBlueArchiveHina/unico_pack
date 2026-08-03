import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    unico_pack = get_package_share_directory('unico_pack')
    # livox_ros_driver2 = get_package_share_directory('livox_ros_driver2')
    
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(unico_pack, 'launch', 'mavros.launch.py')))

    # livox_driver_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(livox_ros_driver2, 'launch_ROS2', 'msg_MID360_launch.py')))

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(unico_pack, 'launch', 'manager_system_bringup.launch.py')))

    nav_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(unico_pack, 'launch', 'custom_bringup_v2.launch.py')))

    first_launch = [
        mavros_launch,
        manager_launch,
        Node(
           package='unico_pack',
           executable='fastlio_to_mavros_only_pose',
           prefix = ['taskset -c 1,2,3']
        ),
    ]

    lifecycle_nodes = [
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='unico_pack',
                    executable='rosbag_lifecycle',
                    prefix = ['taskset -c 1,2,3']
                )
            ]
        ),
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='unico_pack',
                    executable='precision_landing_lifecycle',
                    parameters=[{
                        'use_sim_time': False,
                    }],
                    prefix = ['taskset -c 1,2,3']
                )
            ]
        ),
        TimerAction(
            period=10.0,
            actions=[
                Node(
                    package='unico_pack',
                    executable='livox_lifecycle',
                    prefix = ['taskset -c 1,2,3']
                )
            ]
       ),
        TimerAction(
            period=12.0,
            actions=[
                Node(
                    package='unico_pack',
                    executable='fast_lio_lifecycle_wrapper',
                    prefix = ['taskset -c 1,2,3']
                )
            ]
        ),
        TimerAction(
            period=14.0,
            actions=[
                Node(
                    package='unico_pack',
                    executable='zed_camera_lifecycle',
                    prefix = ['taskset -c 1,2,3']
                )
            ]
       ),
    ]

    

    return LaunchDescription([
        *first_launch,
        *lifecycle_nodes,
        TimerAction(
            period=16.0,
            actions=[
                nav_bringup_launch
            ]
        )

    ])
    
