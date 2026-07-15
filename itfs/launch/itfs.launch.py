from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start only the iTFS driver with one immutable parameter file."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("ilidar_itfs_ros2"), "config", "itfs.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            Node(
                package="ilidar_itfs_ros2",
                executable="ilidar_core",
                name="ilidar_core",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            ),
        ]
    )
