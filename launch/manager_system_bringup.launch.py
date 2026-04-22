from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='unico_pack',
            executable='main.py',
            output='screen',
            parameters=[{
                'home_pose_x': 2.0,
                'home_pose_y': -2.05,
                'rosbag_folder_path': '/home/uni-co-jetson/rosbag',
                'mqtt_broker': '192.168.166.83'
            }
            ],
        ),

        TimerAction(
        period = 3.0,
        actions = [
            Node(
            package='unico_pack',
            executable='mission_dispatcher_v2.py',
            output='screen',
            )]
        ),
        

        TimerAction(
        period = 6.0,
        actions = [
            Node(
                package='unico_pack',
                executable='cmd_vel_bridge',
                prefix=['taskset -c 3']
            ),
            Node(
                package='unico_pack',
                executable='nav_velocity_tracker',
                prefix=['taskset -c 3']
            )]
        ),

        Node(
            package='unico_pack',
            executable='precision_landing_lifecycle',
            prefix = ['taskset -c 3']
        ),

        # Node(
        #     package='unico_pack',
        #     executable='fast_lio_lifecycle_wrapper',
        #     prefix = ['taskset -c 4,5']
        # ),

        Node(
            package='unico_pack',
            executable='rosbag_lifecycle',
            prefix = ['taskset -c 3']
        )
    ])
