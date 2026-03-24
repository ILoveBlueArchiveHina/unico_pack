from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ArUco多標記識別節點
        Node(
            package='aruco_ros',
            executable='marker_publisher', 
            parameters=[{
                'marker_size': 0.03,  # cm
                'reference_frame': 'camera',
                'camera_frame': 'camera',
            }],
            remappings=[
                ('camera_info', 'webcam/camera_info'),
                ('image', 'webcam/image_raw')
            ]
        ),
        
        # 多標記降落控制節點
        Node(
            package='unico_pack',
            executable='multi_marker_landing_node',
        ),
    ])
    
