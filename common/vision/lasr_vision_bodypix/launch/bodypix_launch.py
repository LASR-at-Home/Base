from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare preload argument with default value as a YAML list
    preload_arg = DeclareLaunchArgument(
        'preload', 
        default_value=['resnet50'],
        # default_value="['resnet50', 'mobilenet50']",
        description='Array of models to preload when starting the service'
    )
    
    # Create the BodyPix service node
    bodypix_node = Node(
        package='lasr_vision_bodypix',
        executable='bodypix_services.py',
        name='bodypix_services',
        output='screen',
        parameters=[{'preload': LaunchConfiguration('preload')}],
    )

    # Return the launch description
    return LaunchDescription([
        preload_arg,  # Argument declaration
        bodypix_node  # Node for the BodyPix service
    ])
