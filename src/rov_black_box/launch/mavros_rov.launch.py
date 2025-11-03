from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')

    # Path to mavros launch file
    mavros_launch = PathJoinSubstitution([
        FindPackageShare('mavros'),
        'launch',
        'node.launch'  # or 'mavros.launch.py' depending on your version
    ])
    
    # Path to config files
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
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(mavros_launch),
            launch_arguments={
                'pluginlists_yaml': pluginlists_yaml,
                'config_yaml': config_yaml,
                'fcu_url': fcu_url,
                'gcs_url': gcs_url,
                'tgt_system': tgt_system,
                'tgt_component': tgt_component,
                'log_output': log_output,
                'fcu_protocol': fcu_protocol,
                'respawn_mavros': respawn_mavros,
                'namespace': namespace,
            }.items(),
        ),
    ])

