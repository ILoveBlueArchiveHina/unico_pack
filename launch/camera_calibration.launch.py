"""用來校正相機的launch"""
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # USB camera node (landing cam)
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[{       # 參數根據相機可支援規格調整
                'video_device': '/dev/video0',
                'image_width': 1280,        
                'image_height': 720,        
                'pixel_format': 'mjpeg2rgb',
                'framerate': 10.0,
                'frame_id': "landing_camera",
                'camera_name': 'landing_camera'
            }],
            remappings=[
                ('image_raw', 'landing_camera/image_raw'),
                ('camera_info', 'landing_camera/camera_info')
            ]
        ),

        Node(
            package="camera_calibration",
            executable="cameracalibrator",
            arguments=['--size', '7x9', '--square', '0.02'],
            remappings=[
                ('image', '/landing_camera/image_raw'),
            ],
            parameters=[{
                'camera': '/landing_camera',
            }]
        )
        
    ])
    
