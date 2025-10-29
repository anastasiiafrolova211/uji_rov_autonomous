from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    fcu_url = LaunchConfiguration('fcu_url')
    gcs_url = LaunchConfiguration('gcs_url')
    tgt_system = LaunchConfiguration('tgt_system')
    tgt_component = LaunchConfiguration('tgt_component')
    log_output = LaunchConfiguration('log_output')
    fcu_protocol = LaunchConfiguration('fcu_protocol')
    respawn_mavros = LaunchConfiguration('respawn_mavros')

    # Path to mavros node launch file
    mavros_launch = FindPackageShare('mavros').perform(None) + '/launch/node.launch.py'

    return LaunchDescription([
        DeclareLaunchArgument('namespace', default_value='bluerov2'),
        DeclareLaunchArgument('fcu_url', default_value='udp://192.168.2.1:14550@192.168.2.2'),
        DeclareLaunchArgument('gcs_url', default_value='udp://@127.0.0.1'),
        DeclareLaunchArgument('tgt_system', default_value='1'),
        DeclareLaunchArgument('tgt_component', default_value='1'),
        DeclareLaunchArgument('log_output', default_value='screen'),
        DeclareLaunchArgument('fcu_protocol', default_value='v2.0'),
        DeclareLaunchArgument('respawn_mavros', default_value='false'),

        # Group for MAVROS launch
        GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(mavros_launch),
                launch_arguments={
                    'pluginlists_yaml': FindPackageShare('mavros').perform(None) + '/launch/px4_pluginlists.yaml',
                    'config_yaml': FindPackageShare('mavros').perform(None) + '/launch/px4_config.yaml',
                    'fcu_url': fcu_url,
                    'gcs_url': gcs_url,
                    'tgt_system': tgt_system,
                    'tgt_component': tgt_component,
                    'log_output': log_output,
                    'fcu_protocol': fcu_protocol,
                    'respawn_mavros': respawn_mavros,
                    'namespace': namespace,
                }.items()
            ),
        ]),
    ])
