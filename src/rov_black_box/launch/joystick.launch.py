from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick driver -> publishes /joy
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js1',
                'deadzone': 0.05,
                'autorepeat_rate': 10.0,
            }],
        ),

        # Your BlueROV joystick mapper -> subscribes to /joy, publishes RC override
        Node(
            package='rov_black_box',
            executable='joystick_node',
            name='bluerov_joystick',
            output='screen',
            parameters=[{
                'light_pin': 12.0,
                'gripper_pin': 10.0,
                'camera_servo_pin': 16.0,
            }],
        ),
    ])

