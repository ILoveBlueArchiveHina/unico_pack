import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    unico_pack_path = get_package_share_directory('unico_pack')
    default_params_file = os.path.join(unico_pack_path, 'config', 'custom_bringup_v6.yaml')
    default_map_file = os.path.join(unico_pack_path, 'maps', 'warehouse_2d_map.yaml')
    default_xml_file = os.path.join(unico_pack_path, 'config', 'drone_nav.xml')

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    user = LaunchConfiguration('user')
    map_odom_tf_x = LaunchConfiguration('map_odom_tf_x')
    map_odom_tf_y = LaunchConfiguration('map_odom_tf_y')
    map_odom_tf_z = LaunchConfiguration('map_odom_tf_z')
    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file
    )

    declare_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    declare_map = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file
    )

    declare_user = DeclareLaunchArgument(
        'user',
        default_value="uni-co-jetson"
    )

    declare_map_odom_tf_x = DeclareLaunchArgument(
        'map_odom_tf_x',
        default_value='2.0'
    )

    declare_map_odom_tf_y = DeclareLaunchArgument(
        'map_odom_tf_y',
        default_value='-2.05'
    )

    declare_map_odom_tf_z = DeclareLaunchArgument(
        'map_odom_tf_z',
        default_value='2.05'
    )


    nodes = [
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            prefix=['taskset -c 1,2,3'],
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
            ],
            prefix=['taskset -c 1,2,3'],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            prefix=['taskset -c 1,2,3'],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            prefix=['taskset -c 1,2,3'],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': default_xml_file,
                'default_nav_through_poses_bt_xml': os.path.join(unico_pack_path, 'config', 'drone_nav_through_poses.xml'),
                }],
            prefix=['taskset -c 1,2,3'],
            ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            }],
            prefix=['taskset -c 1,2,3'],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_map',
            output='screen',
            arguments=[
                '--x', map_odom_tf_x,
                '--y', map_odom_tf_y,
                '--z', map_odom_tf_z,
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'map',
                '--child-frame-id', 'camera_init']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_zed',
            output='screen',
            arguments=[
                '--x', '0.05',
                '--y', '0.0',
                '--z', '-0.2',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '0.0',
                '--frame-id', 'body',
                '--child-frame-id', 'zed_camera_link']
        ),

    # TimerAction(
    #     period = 18.0,
    #     actions = [
    #         Node(
    #             package='nav2_velocity_smoother',
    #             executable='velocity_smoother',
    #             name='velocity_smoother',
    #             output='screen',
    #             parameters=[params_file, {'use_sim_time': use_sim_time}],
    #             remappings=[(
    #                 'cmd_vel', 'cmd_vel_raw'),
    #             ],
    #             prefix=['taskset -c 1,2'],
    #         )]
    # )
    ]

    lifecycle_manager = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': True,
                    'node_names': [
                        'map_server',
                        'planner_server',
                        'controller_server',
                        'behavior_server',
                        'waypoint_follower',
                        'bt_navigator',
                        # 'velocity_smoother'
                    ]
                }],
                prefix=['taskset -c 1,2,3'],
            )
        ]
    )
    

    return LaunchDescription([
        declare_params, declare_sim, declare_map, declare_user,
        declare_map_odom_tf_x, declare_map_odom_tf_y, declare_map_odom_tf_z,
    ] + nodes + [lifecycle_manager])
