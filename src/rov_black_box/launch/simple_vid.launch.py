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
            default_value='5602',
            description='UDP port for BlueROV video stream'
        ),

        Node(
            package='rov_black_box',  # Replace with your actual package name
            executable='video_node',    # Replace with your video node executable name
            name='video_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'port': port
            }]
        )
    ])

