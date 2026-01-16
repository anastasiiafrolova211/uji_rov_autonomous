from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    pkg = FindPackageShare("rov_black_box")
    default_aruco_yaml = PathJoinSubstitution([pkg, "config", "aruco_map.yaml"])

    world_frame = LaunchConfiguration("world_frame")
    aruco_yaml = LaunchConfiguration("aruco_map_yaml")

    aruco = Node(
        package="rov_black_box",
        executable="aruco_detector",
        name="aruco_detector",
        output="screen",
        parameters=[
            aruco_yaml,
            {
                "parent_frame": world_frame,
                "publish_aruco_tfs": True,
                "publish_aruco_markers": True,
                "publish_rov_tf": True,
                "publish_rov_marker": True,
                "rov_pose_topic": "/rov/pose",
                "rov_frame": "rov/base_link",
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world_frame", default_value="map"),
        DeclareLaunchArgument("aruco_map_yaml", default_value=default_aruco_yaml),
        aruco,
        rviz,
    ])

