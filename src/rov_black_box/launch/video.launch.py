from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),

        GroupAction([
            # Video stream node (working)
            Node(
                package='rov_black_box',
                executable='video_node',
                name='video_node',
                namespace=namespace,
                output='screen'
            ),

            # Underwater detection with YOLO segmentation
            Node(
                package='rov_black_box',
                executable='underwater_detection_node',
                name='underwater_detection',
                namespace=namespace,
                output='screen',
                parameters=[{
                    'camera_topic': 'camera/image_raw',
                    'yolo_model_path': '/home/elex/uji_rov_autonomous/best.pt',
                    'enable_aruco': True,
                    'enable_yolo': True,
                    'show_visualization': True,
                    'confidence_threshold': 0.5
                }]
            ),
        ])
    ])

