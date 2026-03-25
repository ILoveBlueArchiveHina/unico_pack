from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unico_pack',
            executable='manager.py',
            output='screen',
            parameters=[{
                'home_pose_x': 2.0,
                'home_pose_y': -2.05,
                'rosbag_folder_path': '/home/uni-co-jetson/rosbag'
            }
            ],
            prefix=['taskset -c 3'],
        ),

        TimerAction(
        period = 3.0,
        actions = [
            Node(
            package='unico_pack',
            executable='mission_dispatcher_with_tracker_v2.py',
            output='screen',
            prefix=['taskset -c 3'],
            )]
        ),
        

        TimerAction(
        period = 6.0,
        actions = [
            Node(
                package='unico_pack',
                executable='cmd_vel_bridge.py',
                prefix=['taskset -c 3'],
            )]
        ),
    ])
