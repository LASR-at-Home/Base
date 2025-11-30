import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription


def generate_launch_description():
    return launch.LaunchDescription(
        [
            # Declare launch arguments
            DeclareLaunchArgument(
                "dataset",
                default_value="lab",
                description="Dataset to use for the demo",
            ),
            # Include the face recognition service
            IncludeLaunchDescription(
                launch.launch_description_sources.PythonLaunchDescriptionSource(
                    [
                        FindPackageShare("lasr_vision_deepface"),
                        "/launch/deepface_service_launch.py",
                    ]
                )
            ),
            # Launch V4L2 Camera Node
            launch_ros.actions.Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera_node",
                output="screen",
                parameters=[
                    {
                        "video_device": "/dev/video0",
                        "image_width": 640,
                        "image_height": 480,
                        "pixel_format": "YUYV",
                        "frame_rate": 30,
                    }
                ],
                remappings=[
                    ("/image_raw", "/camera/image_raw")  # Ensure correct topic name
                ],
            ),
            # Launch rqt_image_view to display the camera feed
            launch_ros.actions.Node(
                package="rqt_image_view",
                executable="rqt_image_view",
                name="rqt_image_view",
                output="screen",
            ),
        ]
    )
