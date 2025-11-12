from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Launch configurations
    namespace = LaunchConfiguration('namespace')
    joy_dev = LaunchConfiguration('joy_dev')
    scale_linear = LaunchConfiguration('scale_linear')
    scale_vertical = LaunchConfiguration('scale_vertical')
    scale_yaw = LaunchConfiguration('scale_yaw')
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='bluerov2',
            description='Namespace for all nodes'
        ),
        DeclareLaunchArgument(
            'joy_dev',
            default_value='/dev/input/js1',
            description='Joystick device path'
        ),
        DeclareLaunchArgument(
            'scale_linear',
            default_value='0.3',
            description='Linear movement scale factor (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'scale_vertical',
            default_value='0.4',
            description='Vertical movement scale factor (0.0-1.0)'
        ),
        DeclareLaunchArgument(
            'scale_yaw',
            default_value='0.3',
            description='Yaw rotation scale factor (0.0-1.0)'
        ),

        # Group all nodes under the same namespace
        GroupAction(
            actions=[
                PushRosNamespace(namespace),

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

                Node(
                    package='teleop_twist_joy',
                    executable='teleop_node',
                    name='teleop_twist_joy_node',
                    output='screen',
                    parameters=[{
                        'axis_linear': {'x': 1, 'y': 0, 'z': 4},
                        'scale_linear': {'x': 0.7, 'y': 0.7, 'z': 0.7},
                        'scale_linear_turbo': {'x': 1.0, 'y': 1.0, 'z': 1.0},
                        'axis_angular': {'yaw': 3, 'roll': 7, 'pitch': 6},
                        'scale_angular': {'yaw': 0.4, 'roll': 0.2, 'pitch': 0.2},
                        'require_enable_button': False,
                        'enable_button': 2,
                        'enable_turbo_button': 5,
                    }],
                    remappings=[
                        ('joy', 'joy'),
                        ('cmd_vel', 'cmd_vel'),
                    ]
                ),

                Node(
                    package='rov_black_box',
                    executable='joystick_node',
                    name='joystick_node',
                    output='screen',
                    parameters=[{
                        'update_rate': 50.0,
                        'continuous_rate': 20.0,
                        'scale_linear': scale_linear,
                        'scale_vertical': scale_vertical,
                        'scale_yaw': scale_yaw,
                    }],
                    remappings=[
                        ('cmd_vel', 'cmd_vel'),  # Subscribe to teleop velocity commands
                    ]
                ),
            ]
        )
    ])

