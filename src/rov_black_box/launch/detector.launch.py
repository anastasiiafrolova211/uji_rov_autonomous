from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='box_handle_detector',
            executable='detector_node',
            name='yolo_detector',
            parameters=[
                {'model_path': '/home/elex/uji_rov_autonomous/best_seg.pt'},
                {'conf_threshold': 0.5},
                {'camera_frame': 'camera_link'}
            ],
            output='screen'
        ),
    ])

