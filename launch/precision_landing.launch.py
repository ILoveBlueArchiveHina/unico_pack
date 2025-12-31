from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB camera node (landing cam)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 1920,
                'image_height': 1080,
                'pixel_format': 'mjpeg2rgb',
                'framerate': 30.0,
                'camera_info_url': 'file:///home/uni-co-jetson/ros2_ws/src/unico_pack/config/webcam_param.yaml',
                'frame_id': "camera",
                'camera_name': 'webcam'
            }],
            remappings=[
                ('image_raw', 'webcam/image_raw'),
                ('camera_info', 'webcam/camera_info')
            ]
        ),
        
        # ArUco多標記識別節點
        Node(
            package='aruco_ros',
            executable='marker_publisher', 
            parameters=[{
                'marker_size': 0.09,  # 9 cm
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
            executable='multi_marker_landing.py',
        ),
    ])
