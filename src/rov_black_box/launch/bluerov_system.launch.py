from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    video_port = LaunchConfiguration('video_port')

    mavros_launch = PathJoinSubstitution([
        FindPackageShare('mavros'),
        'launch',
        'node.launch'
    ])
    
    pluginlists_yaml = PathJoinSubstitution([
        FindPackageShare('mavros'),
        'launch',
        'apm_pluginlists.yaml'
    ])
    
    config_yaml = PathJoinSubstitution([
        FindPackageShare('mavros'),
        'launch',
        'apm_config.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument('fcu_url', default_value='udp://192.168.2.1:14550@192.168.2.2'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@127.0.0.1'),
        DeclareLaunchArgument('video_port', default_value='5600'),

        # Launch MAVROS
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                'pluginlists_yaml': pluginlists_yaml,
                'config_yaml': config_yaml,
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'tgt_system': '1',
                'tgt_component': '1',
                'log_output': 'screen',
                'fcu_protocol': 'v2.0',
                'respawn_mavros': 'false',
                'namespace': namespace,
            }.items()
        ),

        # Launch Video Node
        Node(
            package='rov_black_box',
            executable='video_node',
            name='video_node',
            namespace=namespace,
            output='screen',
            parameters=[{
                'port': video_port,
                'enable_detection': True,
                
                # ArUco marker IDs and sizes (MEASURE YOUR MARKERS!)
                'box_marker_id_1': 10,
                'box_marker_id_2': 11,
                'marker_size_1': 0.05,  # 5cm - ADJUST THIS!
                'marker_size_2': 0.08,  # 8cm - ADJUST THIS!
                
                # Box dimensions
                'box_length': 0.30,
                'box_width': 0.16,
                'box_height': 0.14,
                
                # Marker offsets from box center
                'marker1_offset_x': 0.15,
                'marker1_offset_y': 0.0,
                'marker1_offset_z': 0.0,
                'marker2_offset_x': -0.15,
                'marker2_offset_y': 0.0,
                'marker2_offset_z': 0.0,
                
                # Handle offset from box center
                'handle_offset_x': 0.0,
                'handle_offset_y': 0.0,
                'handle_offset_z': 0.07,
            }]
        ),
    ])

