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
                'port': 5600,
                'enable_detection': True,
                
                # ArUco markers
                'aruco_dict': 'DICT_4X4_50',
                'floor_marker_ids': [10, 20],
                'box_marker_id': 42,
                'marker_size': 0.10,
                
                # YOLO model - YOUR PATH
                'yolo_model_path': '/home/elex/uji_rov_autonomous/best_seg.pt',
                'yolo_confidence': 0.5,
                'enable_yolo_detection': True,
            }]
        ),
    ])

