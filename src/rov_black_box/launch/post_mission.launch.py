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
                executable = 'post_mission_node',
                name = 'post_mission_node',
                namespace = namespace,
                output = 'screen'
            )
        ])
    ])


