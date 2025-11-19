from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    port = LaunchConfiguration('port')

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='bluerov2',
            description='Namespace for the video node'
        ),
        DeclareLaunchArgument(
            'port',
            default_value='5600',
            description='UDP port for BlueROV video stream'
        ),

        Node(
            package='rov_black_box',
            executable='video_node',
            name='video_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'port': port,
                'show_window': True      # ensure GUI is enabled
            }]
        )
    ])

