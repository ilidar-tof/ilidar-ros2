from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    """Start the LITE driver with the package's shared parameter file."""
    default_config = PathJoinSubstitution(
        [FindPackageShare("ilidar_lite_ros2"), "config", "lite.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Driver parameter YAML file",
            ),
            Node(
                package="ilidar_lite_ros2",
                executable="ilidar_lite_core",
                name="ilidar_lite_core",
                output="screen",
                parameters=[LaunchConfiguration("config")],
            )
        ]
    )
