import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    unico_pack_path = get_package_share_directory('unico_pack')  # 自己的ros2 套件包位置(share 路徑)
    mavros_pkg_path = get_package_share_directory('mavros')

    pluginlists_yaml = os.path.join(unico_pack_path, 'config', 'mavros_pluginlists.yaml')
    config_yaml = os.path.join(unico_pack_path, 'config', 'apm_config.yaml')


    mavros_node_launch = Node(
        package='mavros',
        executable='mavros_node',
        namespace='mavros',
        output='screen',
        prefix='taskset -c 4,5',
        parameters=[
            {'fcu_url': 'udp://:14550@'},
            {'gcs_url': 'udp://:14555@'},
            {'tgt_system': 1},
            {'tgt_component': 1},
            {'fcu_protocol': 'v2.0'},
            pluginlists_yaml,
            config_yaml,
        ]
    )
    request_extended_state = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call', 
                    '/mavros/set_message_interval', 
                    'mavros_msgs/srv/MessageInterval', 
                    '"{message_id: 245, message_rate: 1.0}"'
                ],
                shell=True,
                output='screen'
            )
        ]
    )


    return LaunchDescription([
        mavros_node_launch,
        request_extended_state
    ])