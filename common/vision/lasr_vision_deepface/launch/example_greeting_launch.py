import launch
import launch.actions
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return launch.LaunchDescription([
        # Declare launch arguments
        launch.actions.DeclareLaunchArgument(
            "debug",
            default_value="true",
            description="Enable debug topic"
        ),

        launch.actions.DeclareLaunchArgument(
            "topic",
            default_value="/xtion/rgb/image_raw",
            description="ROS2 topic for input image stream"
        ),

        launch.actions.DeclareLaunchArgument(
            "dataset",
            default_value="",
            description="Dataset path"
        ),

        # Include the service launch file
        launch.actions.IncludeLaunchDescription(
            launch.launch_description_sources.PythonLaunchDescriptionSource(
                [FindPackageShare("lasr_vision_deepface"), "/launch/deepface_service_launch.py"]
            ),
            launch_arguments={"debug": LaunchConfiguration("debug")}.items(),
        ),

        # Launch the greet node
        launch_ros.actions.Node(
            package="lasr_vision_deepface",
            executable="greet",
            name="greet",
            output="screen",
            arguments=[
                LaunchConfiguration("topic"),
                LaunchConfiguration("dataset")
            ]
        ),
    ])
