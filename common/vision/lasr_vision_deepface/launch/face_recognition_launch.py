import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # Declare a launch argument for enabling debug mode
        launch.actions.DeclareLaunchArgument(
            "debug",
            default_value="false",
            description="Whether to publish plotted images to /recognise/debug",
        ),

        # Start the face recognition service node
        launch_ros.actions.Node(
            package="lasr_vision_deepface",
            executable="service",  # The actual executable file
            name="face_recognition_service",
            output="screen",
            parameters=[{"debug": launch.substitutions.LaunchConfiguration("debug")}]
        ),
    ])
