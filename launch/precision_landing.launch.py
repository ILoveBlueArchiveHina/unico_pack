from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import TimerAction

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
                'framerate': 5.0,
                'camera_info_url': 'file:///home/uni-co-jetson/ros2_ws/src/unico_pack/config/arducam_fisheye.yaml',
                'frame_id': "camera",
                'camera_name': 'landing_cam'
            }],
            remappings=[
                ('image_raw', 'landing_cam/image_raw'),
                ('camera_info', 'landing_cam/camera_info')
            ],
            prefix=['nice -n 10 '],
        ),
        
        # ArUco多標記識別節點
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='aruco_ros',
                    executable='marker_publisher', 
                    parameters=[{
                        'marker_size': 0.03,  # cm
                        'reference_frame': 'camera',
                        'camera_frame': 'camera',
                    }],
                    remappings=[
                        ('camera_info', 'landing_cam/camera_info'),
                        ('image', 'landing_cam/image_raw')
                    ],
                    prefix=['nice -n 10 '],
                ),

                # 多標記降落控制節點
                Node(
                    package='unico_pack',
                    executable='multi_marker_landing_node',
                    prefix=['nice -n 10 '],
                ),
            ]
        ),
        
    ])
    
