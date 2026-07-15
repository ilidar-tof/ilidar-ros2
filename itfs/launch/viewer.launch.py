from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the iTFS driver and one supervised multi-device RViz process."""
    share = FindPackageShare("ilidar_itfs_ros2")
    config = LaunchConfiguration("config")
    viewer = Node(
        package="ilidar_itfs_ros2",
        executable="ilidar_viewer",
        name="ilidar_viewer",
        output="screen",
        parameters=[{
            "sensor_sns": ParameterValue(LaunchConfiguration("sensor_sns"), value_type=str),
            "fixed_frame": ParameterValue(LaunchConfiguration("fixed_frame"), value_type=str),
            "rviz_template": ParameterValue(
                PathJoinSubstitution([share, "rviz", "itfs.rviz"]), value_type=str),
        }],
    )
    return LaunchDescription([
        DeclareLaunchArgument("sensor_sns", default_value="",
                              description="Optional SN list; empty discovers /ilidar_<SN>/info"),
        DeclareLaunchArgument("fixed_frame", default_value="base_link"),
        DeclareLaunchArgument("config", default_value=PathJoinSubstitution([share, "config", "itfs.yaml"])),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([share, "launch", "itfs.launch.py"])),
            launch_arguments={"config": config}.items()),
        viewer,
        RegisterEventHandler(OnProcessExit(
            target_action=viewer,
            on_exit=[EmitEvent(event=Shutdown(reason="iTFS viewer exited; stopping driver"))])),
    ])
