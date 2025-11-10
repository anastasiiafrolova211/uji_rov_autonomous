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
                
                # ArUco markers - 4x4 dictionary
                'aruco_dict': 'DICT_4X4_50',        # 4x4 pattern
                'floor_marker_ids': [10, 20],       # Floor marker IDs
                'box_marker_id': 42,                # Box marker (if different)
                'marker_size': 0.10,                # 10cm = 0.10 meters (BLACK ARUCO ONLY)
                
                # Detection flags
                'enable_box_detection': True,
                'enable_handle_detection': True,
            }]
        ),
    ])

