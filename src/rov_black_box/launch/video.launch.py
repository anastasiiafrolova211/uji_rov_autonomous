from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),


        GroupAction([
            Node(
                package = 'rov_black_box',
                executable = 'video_node',
                name = 'video_node',
                namespace = namespace,
                output = 'screen'
            ),

            Node(
                package = 'rov_black_box',
                executable = 'track_box_node',
                name = 'track_box_node',
                namespace = namespace,
                output = 'screen'
            )
        ])
    ])


