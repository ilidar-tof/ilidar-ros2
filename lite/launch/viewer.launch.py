from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the driver and one supervised RViz instance."""
    package_share = FindPackageShare("ilidar_lite_ros2")
    lite_launch_file = PathJoinSubstitution(
        [package_share, "launch", "lite.launch.py"]
    )
    default_config = PathJoinSubstitution([package_share, "config", "lite.yaml"])
    rviz_template = PathJoinSubstitution([package_share, "rviz", "lite.rviz"])

    sensor_sns = LaunchConfiguration("sensor_sns")
    fixed_frame = LaunchConfiguration("fixed_frame")
    config = LaunchConfiguration("config")

    viewer = Node(
        package="ilidar_lite_ros2",
        executable="ilidar_lite_viewer",
        name="ilidar_lite_viewer",
        output="screen",
        parameters=[
            {
                "sensor_sns": ParameterValue(sensor_sns, value_type=str),
                "fixed_frame": ParameterValue(fixed_frame, value_type=str),
                "rviz_template": ParameterValue(rviz_template, value_type=str),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "sensor_sns",
                default_value="",
                description="Optional SN list; empty discovers /ilidar_lite_<SN>/info topics",
            ),
            DeclareLaunchArgument(
                "fixed_frame",
                default_value="base_link",
                description="RViz fixed and target frame",
            ),
            DeclareLaunchArgument(
                "config",
                default_value=default_config,
                description="Driver parameter YAML file",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(lite_launch_file),
                launch_arguments={"config": config}.items(),
            ),
            viewer,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=viewer,
                    on_exit=[
                        EmitEvent(
                            event=Shutdown(
                                reason="iLidar LITE viewer exited; stopping driver"
                            )
                        )
                    ],
                )
            ),
        ]
    )
