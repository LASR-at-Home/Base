from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera_node",
                output="screen",
                parameters=[
                    {
                        "video_device": "/dev/video0",  # the video device to open
                        "image_width": 640,
                        "image_height": 480,
                        "pixel_format": "YUYV",  # the pixel format of the image
                        "frame_rate": 30,
                    }
                ],
            ),
        ]
    )
