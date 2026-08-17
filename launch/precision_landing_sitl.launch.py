from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ArUco多標記識別節點
        Node(
            package='aruco_ros',
            executable='marker_publisher', 
            parameters=[{
                'marker_size': 0.1,  # cm
                'reference_frame': 'landing_camera',
                'camera_frame': 'landing_camera',
            }],
            remappings=[
                ('camera_info', 'landing_camera/camera_info'),
                ('image', 'landing_camera/image_raw')
            ]
        ),
        
        # 多標記降落控制節點
        Node(
            package='unico_pack',
            executable='precision_landing',
        ),
    ])
    
