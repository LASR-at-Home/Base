from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            output='screen',
            parameters=[{
                'video_device': '/dev/video0',    # 摄像头设备路径
                'image_width': 640,              # 图像宽度
                'image_height': 480,             # 图像高度
                'pixel_format': 'YUYV',          # 像素格式
                'frame_rate': 30,                # 帧率
            }],
        ),
    ])
