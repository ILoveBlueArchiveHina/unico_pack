from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unico_pack',
            executable='manager.py',
            output='screen',
            parameters=[{
                'home_pose_x': 0.0,
                'home_pose_y': 0.0,
                'rosbag_folder_path': '/home/uni-co-jetson/rosbag'
            }
            ],
            prefix=['nice -n 10 '],
        ),

        Node(
            package='unico_pack',
            executable='mission_dispatcher_with_tracker_v2.py',
            output='screen',
            prefix=['nice -n 10 '],
        )
    ])
