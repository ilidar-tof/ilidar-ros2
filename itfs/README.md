# iTFS ROS 2 Package

ROS 2 receive-only bridge for the `iTFS::LiDAR` C++ API.

Version: `V2.0.0`

## Compatibility

|Sensor|Minimum firmware|ROS package|
|:---|:---:|:---:|
|iTFS-110|V1.5.0|`ilidar_itfs_ros2`|
|iTFS-80|V1.5.0|`ilidar_itfs_ros2`|

|Ubuntu|ROS 2|C++ baseline|Validation|
|:---:|:---:|:---:|:---|
|20.04|Foxy|C++17|Build verified|
|22.04|Humble|C++17|Build verified|
|24.04|Jazzy|C++17|Build and sensor runtime verified|
|26.04|Lyrical Luth|TBD|Planned|

> [!WARNING]
> Foxy has reached end of life and is retained for existing Ubuntu 20.04
> deployments.

> [!NOTE]
> Support for ROS 2 Lyrical Luth on Ubuntu 26.04 LTS is planned.

## Quick Start

From a ROS 2 workspace containing this package:

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ilidar_itfs_ros2
source install/setup.bash
ros2 launch ilidar_itfs_ros2 viewer.launch.py
```

The viewer discovers `/ilidar_<SN>/info` topics and creates one RViz2 group per
detected serial number. Closing RViz2 also stops the included driver.

## Dependencies

Dependencies are declared in `package.xml`. The driver uses `rclcpp`, standard
ROS 2 messages, `tf2_ros`, `ament_index_cpp`, and pthreads. The viewer
additionally uses `rclpy`, RViz2, and PyYAML. OpenCV, PCL, and Open3D are not
required.

## Run and Verify

```bash
# Driver only
ros2 launch ilidar_itfs_ros2 itfs.launch.py

# Driver and RViz2 with automatic discovery
ros2 launch ilidar_itfs_ros2 viewer.launch.py

# Create known RViz groups immediately
ros2 launch ilidar_itfs_ros2 viewer.launch.py sensor_sns:=2405,2406
```

An explicit `sensor_sns` list configures RViz2 only; it does not filter devices
accepted by the driver. Use `fixed_frame:=<frame>` when the sensors use a
parent other than `base_link`.

Replace `2405` with the detected decimal serial number:

```bash
ros2 node list
ros2 topic list | grep ilidar
ros2 topic echo /ilidar_2405/info --once
ros2 topic echo /ilidar_2405/depth/image_raw --once --field encoding
ros2 topic echo /diagnostics --once
```

## Topics, QoS, and Data Contract

Each device publishes below `/ilidar_<SN>`.

|Topic|Type|QoS|Description|
|:---|:---|:---|:---|
|`depth/image_raw`|`sensor_msgs/msg/Image`|Sensor data|Depth image|
|`intensity/image_raw`|`sensor_msgs/msg/Image`|Sensor data|Intensity or capture-mode-0 gray image|
|`points`|`sensor_msgs/msg/PointCloud2`|Sensor data|Organized calibrated XYZ with optional intensity|
|`info`|`std_msgs/msg/String`|Reliable, transient local|Sensor summary|
|`/diagnostics`|`diagnostic_msgs/msg/DiagnosticArray`|Reliable|Aggregated sensor status|
|`/tf_static`|`tf2_msgs/msg/TFMessage`|Static TF|Mount and optical transforms|

External image and point-cloud subscribers should use a sensor-data-compatible
profile, normally best-effort reliability with volatile durability. Sensor
topics may not connect to a reliable-only subscriber. The `info` topic uses
transient-local durability so late subscribers receive the startup summary.

Example CLI override when required:

```bash
ros2 topic echo /ilidar_2405/depth/image_raw \
  --qos-reliability best_effort --once
```

|Output|Encoding / fields|Unit and invalid-value rule|
|:---|:---|:---|
|Depth|`mono16`|Unsigned millimeters; zero means invalid/no range|
|Intensity / gray|`mono16`|Original unsigned sensor value|
|Point cloud|Organized `x/y/z` `float32`|Meters; invalid points are `NaN`|
|Optional point field|`intensity` `float32`|Converted sensor intensity|

Products from one completed frame share one host ROS timestamp. Topics are
advertised only when their `publish_*` parameter is enabled and may remain
silent when the configured sensor output does not contain that product.

### Capture Modes

|Capture mode|Image layout|ROS behavior|
|:---:|:---|:---|
|0|`240 x 320` gray|Intensity topic only|
|1–3|`capture_row x 320` depth with optional intensity|Depth|

Capture mode, row count, `data_output`, model, and serial number are fixed when
the device context is created. Restart the driver after changing them.

## Mapping, Frames, and TF

PointCloud2 uses a calibrated direction table. The default is `iTFS-110.dat`;
select `iTFS-80.dat` or a custom table with `mapping_file`. If loading fails,
images remain available while PointCloud2 is disabled.

The direction table is not a pinhole camera model, so this package does not
publish CameraInfo.

```text
base_link
  -> ilidar_<SN>_link
      -> ilidar_<SN>_optical_frame
```

Images use the optical frame and PointCloud2 uses the link frame. Translations
are meters and rotations are roll/pitch/yaw radians. Set `publish_tf: false`
when another component owns the transforms.

## Parameters

|Parameter|Default|Description|
|:---|:---|:---|
|`broadcast_ip`|empty|SDK broadcast-interface override|
|`listening_ip`|empty|SDK receive-interface override|
|`listening_port`|`7256`|UDP receive port|
|`mapping_file`|installed `iTFS-110.dat`|Direction table|
|`parent_frame_id`|`base_link`|Mount parent frame|
|`link_frame_id`|SN-derived|Point-cloud frame|
|`frame_id`|SN-derived|Image optical frame|
|`publish_tf`|`true`|Publish static transforms|
|`publish_depth`|`true`|Publish depth when available|
|`publish_intensity`|`true`|Publish intensity or gray|
|`publish_pointcloud`|`true`|Publish organized points|
|`publish_pointcloud_intensity`|`true`|Add optional intensity field|
|`publish_info`|`true`|Publish sensor summary|
|`publish_diagnostics`|`true`|Publish diagnostics|
|`sensor_qos_depth`|`5`|Sensor-data history depth|
|`diagnostics_qos_depth`|`10`|Diagnostics history depth|

Pose parameters use `mount_translation_{x,y,z}`,
`mount_rotation_{roll,pitch,yaw}`, `optical_translation_{x,y,z}`, and
`optical_rotation_{roll,pitch,yaw}`. Parameters are read-only during a run.

## Configuration Examples

Single iTFS-80 with a measured mount offset:

```yaml
ilidar_core:
  ros__parameters:
    parent_frame_id: base_link
    devices:
      ilidar_2405:
        mapping_file: /absolute/path/iTFS-80.dat
        mount_translation_x: 0.25
        mount_translation_z: 0.80
```

Multiple sensors inherit common defaults and override only their mount poses:

```yaml
ilidar_core:
  ros__parameters:
    devices:
      ilidar_2405:
        mount_translation_y: 0.15
      ilidar_2406:
        mount_translation_y: -0.15
```

Disable package-owned TF when transforms come from another component:

```yaml
ilidar_core:
  ros__parameters:
    publish_tf: false
```

Pass an external configuration file with:

```bash
ros2 launch ilidar_itfs_ros2 itfs.launch.py \
  config:=/absolute/path/itfs.yaml
```

## Diagnostics and Performance

Each initialized sensor contributes `ilidar_<SN>/status` to `/diagnostics`.
Diagnostics include identity, capture layout, sensor warnings, temperature,
power, packet error counters, mapping status, frame IDs, and
`ros_frame_overwrite_count`.

- `OK`: status is present with no active warning.
- `WARN`: waiting for status, sensor/frame warning, missing rows, or missing mapping.
- `ERROR`: sensor identity or immutable capture layout changed.

The receive callback copies requested data into a reusable latest-frame
snapshot. Each sensor has one worker. If processing falls behind, newer input
replaces pending work and `ros_frame_overwrite_count` increases. DDS history
depth does not change this internal policy.

## Limitations and Troubleshooting

- Sensor configuration and maintenance operations remain external.
- Runtime identity, capture layout, and ROS parameter changes require restart.
- Viewer auto-discovery does not add sensors after RViz2 has started.
- CameraInfo is unavailable because the mapping table is not a pinhole model.
- The latest-frame policy is not suitable for lossless recording.

If no sensor topics appear, verify the sensor stream, IPv4 configuration, UDP
port `7256`, firewall, and interface overrides. If PointCloud2 is absent, check
depth output, `publish_pointcloud`, and `mapping_file`. For TF errors, confirm
the RViz2 fixed frame and unique per-sensor child frames.

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for release history.

## License

This package and its included SDK source are licensed under the MIT License.
See [LICENSE](LICENSE). Third-party ROS 2 and system dependencies retain their
own licenses.
