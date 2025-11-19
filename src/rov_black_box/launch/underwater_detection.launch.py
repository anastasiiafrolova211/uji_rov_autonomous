from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rov_black_box',
            executable='underwater_detection_node',
            name='underwater_detection',
            output='screen',
            parameters=[{
                # Camera topic from your image source / bridge
                'camera_topic': '/bluerov2/camera/image_raw',

                # YOLO model
                'yolo_model_path': '/home/elex/uji_rov_autonomous/v8m_1280.pt',
                'enable_yolo': True,

                # Class-specific thresholds (tune as needed)
                'box_conf_threshold': 0.5,
                'handle_conf_threshold': 0.3,

                # ArUco (true if no aruco detection)
                'enable_aruco': True,

                # GUI window
                'show_visualization': True,
            }]
        ),
    ])

