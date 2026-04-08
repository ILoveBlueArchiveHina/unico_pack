from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map_file')
    user = LaunchConfiguration('user')

    
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value='/home/uni-co-jetson/ros2_ws/src/unico_pack/config/custom_bringup_v5.yaml'
    )

    declare_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false'
    )

    declare_map = DeclareLaunchArgument(
        'map_file',
        default_value="/home/uni-co-jetson/ros2_ws/src/unico_pack/maps/warehouse_2d_map.yaml"
    )

    declare_user = DeclareLaunchArgument(
        'user',
        default_value="uni-co-jetson"
    )

    nodes = [
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            prefix=['taskset -c 1,2'],
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
            prefix=['taskset -c 1,2'],
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            prefix=['taskset -c 1,2'],
        ),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[params_file, {'use_sim_time': use_sim_time}],
            prefix=['taskset -c 1,2'],
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[params_file, {
                'use_sim_time': use_sim_time,
                'default_nav_to_pose_bt_xml': PathJoinSubstitution(["/home", user, "ros2_ws/src/unico_pack/config/drone_nav.xml"]),
                'default_nav_through_poses_bt_xml': PathJoinSubstitution(["/home", user, "ros2_ws/src/unico_pack/config/drone_nav_through_poses.xml"])
                }],
            prefix=['taskset -c 1,2'],
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
            prefix=['taskset -c 1,2'],
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
                    'autostart': False,
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
                prefix=['taskset -c 1,2'],
            )
        ]
    )
    

    return LaunchDescription([declare_params, declare_sim, declare_map, declare_user] + nodes + [lifecycle_manager])
