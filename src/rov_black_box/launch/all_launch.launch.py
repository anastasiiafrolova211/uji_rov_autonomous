from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    launch_dir = os.path.dirname(os.path.realpath(__file__))

    
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'mavros_rov.launch.py'))
    )
    joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'joystick.launch.py'))
    )
    video_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'video.launch.py'))
    )
    post_mission_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'post_mission.launch.py'))
    )

    return LaunchDescription([
        mavros_launch,
        joystick_launch,
        video_launch,
        post_mission_launch,
    ])
