from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
                'mqtt_broker': 'broker.emqx.io'
            }],
            prefix=['taskset -c 1,2,3']
        ),

        TimerAction(
        period = 3.0,
        actions = [
            Node(
            package='unico_pack',
            executable='mission_dispatcher_v4.py',
            parameters=[{
                'tracking_mode': True,
            }],
            output='screen',
            prefix=['taskset -c 1,2,3']
            )]
        ),
        

        TimerAction(
        period = 6.0,
        actions = [
            Node(
                package='unico_pack',
                executable='cmd_vel_bridge',
                prefix=['taskset -c 1,2,3']
            ),
            Node(
                package='unico_pack',
                executable='velocity_controller',
                prefix=['taskset -c 1,2,3']
            )]
        ),

        Node(
            package='unico_pack',
            executable='precision_landing_lifecycle',
            prefix = ['taskset -c 1,2,3']
        ),

        # Node(
        #     package='unico_pack',
        #     executable='fast_lio_lifecycle_wrapper',
        #     prefix = ['taskset -c 4,5']
        # ),

        Node(
            package='unico_pack',
            executable='rosbag_lifecycle',
            prefix = ['taskset -c 1,2,3']
        ),

        
    ])
