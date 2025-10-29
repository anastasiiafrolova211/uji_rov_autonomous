from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    joy_config = LaunchConfiguration('joy_config')
    joy_dev = LaunchConfiguration('joy_dev')
    config_filepath = LaunchConfiguration('config_filepath')
    joy_topic = LaunchConfiguration('joy_topic')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument('joy_config', default_value='xbox'),
        DeclareLaunchArgument('joy_dev', default_value='/dev/input/js0'),
        DeclareLaunchArgument(
            'config_filepath',
            default_value=[FindPackageShare('rov_black_box'), '/config/', joy_config, '.config.yaml']
        ),
        DeclareLaunchArgument('joy_topic', default_value='joy'),

        GroupAction([
            Node(
                package='joy',
                executable='joy_node', # driver for gamepad
                name='joy_node',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'dev': joy_dev,
                    'deadzone': 0.2,
                    'autorepeat_rate': 0.0
                }],
                remappings=[('/joy', joy_topic)]
            ),

            Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                namespace=namespace,
                output='screen',
                parameters=[config_filepath]
            ),

            Node(
                package='rov_black_box',
                executable='joystick_node',
                name='bluerov_custom_mapper',
                namespace=namespace,
                output='screen'
            )
        ])
    ])
