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

    # MAVROS launch file and config
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
        # Launch Arguments
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument('fcu_url', default_value='udp://192.168.2.1:14550@192.168.2.2'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@127.0.0.1'),
        DeclareLaunchArgument('video_port', default_value='5600'),

        # MAVROS Node
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

        # YOLO Detector Node
        Node(
            package='box_handle_detector',
            executable='detector_node',
            name='yolo_detector',
            namespace=namespace,
            output='screen',
            parameters=[{
                'model_path': '/home/ubuntu/models/box_handle_best.pt',  # Update path to your model
                'conf_threshold': 0.5,
                'camera_frame': 'camera_link'
            }]
        ),
    ])

