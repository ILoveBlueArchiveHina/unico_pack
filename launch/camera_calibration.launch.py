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
                'image_width': 1280,
                'image_height': 720,
                'pixel_format': 'mjpeg2rgb',
                'framerate': 10.0,
                'frame_id': "camera",
                'camera_name': 'webcam'
            }],
            remappings=[
                ('image_raw', 'webcam/image_raw'),
                ('camera_info', 'webcam/camera_info')
            ]
        )
        
    ])
    
