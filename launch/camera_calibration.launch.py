from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # # USB相機節點
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 1920,
                'image_height': 1080,
                'framerate': 30.0,
                # 'camera_info_url': 'file:///home/user/.ros/camera_info/camera.yaml'
            }],
            remappings=[
                ('image_raw', 'camera/image_raw'),
                ('camera_info', 'camera/camera_info')
            ]
        ),
        
        # ArUco多標記識別節點
        # Node(
        #     package='aruco_ros',
        #     executable='marker_publisher',  # 多標記版本
        #     parameters=[{
        #         'marker_size': 0.10,  # 10公分
        #         'reference_frame': 'crazyflie/base_footprint/camera',
        #         'camera_frame': 'crazyflie/base_footprint/camera',
        #     }],
        #     remappings=[
        #         ('camera_info', '/camera_info'),
        #         ('image', '/image_raw')
        #     ]
        # ),
        
        # 多標記降落控制節點
        # Node(
        #     package='drone_in_warehouse',
        #     executable='multi_marker_landing.py',
        # ),
    ])
