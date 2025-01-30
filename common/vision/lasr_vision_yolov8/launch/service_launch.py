from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess

def generate_launch_description():
    preload = DeclareLaunchArgument(
        'preload',
        default_value=[],
    )
    yolo_service = Node(
            package='lasr_vision_yolov8',
            executable='yolo_service',
            name='yolo_service',
            parameters='models.yaml'
        )
    change_preload_arg = ExecuteProcess(
        cmd = [[
            'ros2 param set',
            '/~preload',
            preload,
        ]],
        output='screen',
        shell=True
    )
    return LaunchDescription([
        preload, yolo_service, change_preload_arg
    ])