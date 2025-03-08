import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    return launch.LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            "dataset",
            default_value="lab",
            description="Dataset to use for the demo",
        ),

        # Include the face recognition service
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare("lasr_vision_deepface"), "/launch/greet_launch.py"
            ])
        ),

        # Launch rqt_image_view to display the debug topic
        launch_ros.actions.Node(
            package="rqt_image_view",
            executable="rqt_image_view",
            name="image_view",
            output="screen",
            arguments=["/recognise/debug"],
        ),

        # Launch relay service
        launch_ros.actions.Node(
            package="lasr_vision_deepface",
            executable="relay",
            name="relay",
            output="screen",
            arguments=["/camera/image_raw", LaunchConfiguration("dataset")],
        ),

        # Launch V4L2 Camera Node (instead of video_stream_opencv)
        launch_ros.actions.Node(
            package="v4l2_camera",
            executable="v4l2_camera_node",
            name="v4l2_camera_node",
            output="screen",
            parameters=[
                {
                    "video_device": "/dev/video0",  # The video device to open
                    "image_width": 640,
                    "image_height": 480,
                    "pixel_format": "YUYV",  # The pixel format of the image
                    "frame_rate": 30,
                }
            ],
        ),
    ])
