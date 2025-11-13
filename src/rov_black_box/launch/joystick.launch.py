from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configurations
    joy_dev = LaunchConfiguration('joy_dev')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js0',
            description='Joystick device path'
        ),

        # Joy node - reads gamepad and publishes Joy messages
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.05,
                'autorepeat_rate': 50.0,
            }]
        ),

        # Custom BlueROV joystick node with depth hold
        Node(
            package='rov_black_box',  # Your package name
            executable='joystick_node',  # Your node executable name
            name='bluerov_joystick_depth_hold_node',
            output='screen',
        ),
    ])

