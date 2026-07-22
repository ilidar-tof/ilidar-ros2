# iTFS-LITE ROS 2 Package

ROS 2 receive-only bridge for the `iTFS::LITE` C++ API.

Version: `V2.0.1`

## Compatibility

|Sensor|Minimum firmware|ROS package|
|:---|:---:|:---:|
|iTFS-LITE|V1.0.0|`ilidar_lite_ros2`|

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
colcon build --packages-select ilidar_lite_ros2
source install/setup.bash
ros2 launch ilidar_lite_ros2 viewer.launch.py
```

The viewer discovers `/ilidar_lite_<SN>/info` topics and creates one RViz2
group per detected serial number. Closing RViz2 also stops the included driver.

## Dependencies

Dependencies are declared in `package.xml`. The driver uses `rclcpp`, standard
ROS 2 messages, `tf2_ros`, and pthreads. The viewer additionally uses `rclpy`,
RViz2, and PyYAML. OpenCV, PCL, and Open3D are not required.

## Run and Verify

```bash
# Driver only
ros2 launch ilidar_lite_ros2 lite.launch.py

# Driver and RViz2 with automatic discovery
ros2 launch ilidar_lite_ros2 viewer.launch.py

# Create known RViz groups immediately
ros2 launch ilidar_lite_ros2 viewer.launch.py sensor_sns:=87,88,101
```

An explicit `sensor_sns` list configures RViz2 only; it does not filter devices
accepted by the driver. Use `fixed_frame:=<frame>` when the sensors use a
parent other than `base_link`.

Replace `87` with the detected decimal serial number:

```bash
ros2 node list
ros2 topic list | grep ilidar_lite
ros2 topic echo /ilidar_lite_87/info --once
ros2 topic echo /ilidar_lite_87/depth/image_raw --once --field encoding
ros2 topic echo /diagnostics --once
```

## Recommended Sensor Output

|Data|Recommended input|ROS output|Reason|
|:---|:---|:---|:---|
|Depth / XYZ-Z|`DEPTH_MM_16` or `XYZ_MM_16`|`mono16` millimeters|No depth restoration|
|Amplitude|`RAW_16`|`mono16` raw scale|No lossy restoration|
|Intensity|`RAW_16`|`mono16` raw scale|No lossy restoration|
|Confidence|`MASK_1`|`mono8`, `0`/`255`|Matches the binary contract|

Other supported modes are converted to the same ROS outputs but may require
extra work or may have lost sensor-side precision. A stream set to `OFF`
produces no corresponding messages.

## Topics, QoS, and Data Contract

Each device publishes below `/ilidar_lite_<SN>`.

|Topic|Type|QoS|Description|
|:---|:---|:---|:---|
|`depth/image_raw`|`sensor_msgs/msg/Image`|Sensor data|Restored depth or XYZ-Z|
|`depth/camera_info`|`sensor_msgs/msg/CameraInfo`|Sensor data|Auxiliary LITE intrinsics|
|`amplitude/image_raw`|`sensor_msgs/msg/Image`|Sensor data|Restored amplitude|
|`intensity/image_raw`|`sensor_msgs/msg/Image`|Sensor data|Restored intensity|
|`confidence/image_raw`|`sensor_msgs/msg/Image`|Sensor data|Binary validity mask|
|`points`|`sensor_msgs/msg/PointCloud2`|Sensor data|Organized XYZ with optional amplitude|
|`info`|`std_msgs/msg/String`|Reliable, transient local|Sensor summary|
|`/diagnostics`|`diagnostic_msgs/msg/DiagnosticArray`|Reliable|Aggregated sensor status|

External image and point-cloud subscribers should use a sensor-data-compatible
profile, normally best-effort reliability with volatile durability. Sensor
topics may not connect to a reliable-only subscriber. The `info` topic uses
transient-local durability so late subscribers receive the startup summary.

Example CLI override when required:

```bash
ros2 topic echo /ilidar_lite_87/depth/image_raw \
  --qos-reliability best_effort --once
```

|Output|Encoding / fields|Unit and invalid-value rule|
|:---|:---|:---|
|Depth|`mono16`|Unsigned millimeters; zero means invalid/no range|
|Amplitude|`mono16`|Restored sensor raw scale|
|Intensity|`mono16`|Restored sensor raw scale|
|Confidence|`mono8`|`0` invalid, `255` valid|
|Point cloud|Organized `x/y/z` `float32`|Meters; invalid points are `NaN`|
|Optional point field|`amplitude` `float32`|Same scale as the amplitude image|

All images are `320 x 240`. Products from one completed frame share one host
ROS timestamp. Conversion is subscriber-driven.

## Image Conversion

|LITE input|Depth conversion|
|:---|:---|
|`DEPTH_MM_16`|Use received millimeters|
|`DEPTH_RAW_Q16`|Convert Q16 range to millimeters|
|`DEPTH_LIN_8`|Restore linear range to millimeters|
|`DEPTH_LOG_8`|Restore logarithmic range through the SDK LUT|
|`XYZ_MM_16`|Use received Z millimeters|
|`XYZ_RAW_Q15_Q16`|Convert Q16 Z to millimeters|
|`XYZ_LIN_8`|Restore linear Z to millimeters|

RAW16 amplitude and intensity are retained; LIN8 and LOG8 inputs are restored
to a 16-bit scale. Confidence MASK1 maps to `0`/`255`; RAW16 values below `16`
map to invalid and values at least `16` map to valid. Conversion cannot recover
precision already discarded by a sensor-side 8-bit mode.

## Frames and CameraInfo

```text
base_link
  -> ilidar_lite_<SN>_link
      -> ilidar_lite_<SN>_optical_frame
```

Images and CameraInfo use the optical frame; PointCloud2 uses the link frame.
Translations are meters and rotations are roll/pitch/yaw radians. Set
`publish_tf: false` when another component owns the transforms.

CameraInfo is published after the SDK reconstruction map becomes valid. It
uses ROS `plumb_bob`; a nonzero SDK `k4*r^8` coefficient cannot be represented
and produces a warning. Use PointCloud2 when exact SDK 3D reconstruction is
required.

## Parameters

|Parameter|Default|Description|
|:---|:---|:---|
|`broadcast_ip`|empty|SDK broadcast-interface override|
|`listening_ip`|empty|SDK receive-interface override|
|`listening_port`|`7256`|UDP receive port|
|`parent_frame_id`|`base_link`|Mount parent frame|
|`link_frame_id`|SN-derived|Point-cloud frame|
|`frame_id`|SN-derived|Image optical frame|
|`publish_tf`|`true`|Publish static transforms|
|`publish_depth`|`true`|Publish depth|
|`publish_amplitude`|`true`|Publish amplitude when available|
|`publish_intensity`|`true`|Publish intensity when available|
|`publish_confidence`|`true`|Publish confidence when available|
|`publish_camera_info`|`true`|Publish CameraInfo when valid|
|`publish_pointcloud`|`true`|Publish organized points|
|`publish_pointcloud_amplitude`|`true`|Add optional amplitude field|
|`publish_info`|`true`|Publish sensor summary|
|`publish_diagnostics`|`true`|Publish diagnostics|
|`sensor_qos_depth`|`5`|Sensor-data history depth|
|`diagnostics_qos_depth`|`10`|Diagnostics history depth|

Pose parameters use `mount_translation_{x,y,z}`,
`mount_rotation_{roll,pitch,yaw}`, `optical_translation_{x,y,z}`, and
`optical_rotation_{roll,pitch,yaw}`. Parameters are read-only during a run.

## Configuration Examples

Single sensor with a measured mount pose:

```yaml
ilidar_lite_core:
  ros__parameters:
    parent_frame_id: base_link
    devices:
      ilidar_lite_87:
        mount_translation_x: 0.25
        mount_translation_z: 0.80
        mount_rotation_yaw: 0.0
```

Multiple sensors can override poses and outputs independently:

```yaml
ilidar_lite_core:
  ros__parameters:
    devices:
      ilidar_lite_87:
        mount_translation_y: 0.15
      ilidar_lite_88:
        mount_translation_y: -0.15
        publish_amplitude: false
```

Disable package-owned TF when transforms come from another component:

```yaml
ilidar_lite_core:
  ros__parameters:
    publish_tf: false
```

Pass an external configuration file with:

```bash
ros2 launch ilidar_lite_ros2 lite.launch.py \
  config:=/absolute/path/lite.yaml
```

Restart the driver after changing YAML, sensor `data_output`, or other sensor
layout values.

## Diagnostics and Performance

Diagnostics include identity, input modes, sensor warnings, temperature,
power, packet error counters, conversion warnings, frame IDs, and
`ros_frame_overwrite_count`.

- `OK`: no active warning and native ROS input modes are used.
- `WARN`: sensor/frame warning, missing rows, or non-native input conversion.
- `ERROR`: sensor identity or `data_output` changed after initialization.

Each sensor has one worker and reusable latest-frame buffers. New input
replaces pending work when conversion cannot keep up, increasing
`ros_frame_overwrite_count`. DDS history depth does not change this internal
policy. Disable unused publishers and prefer native sensor modes to reduce load.

## Limitations and Troubleshooting

- Sensor configuration and maintenance operations remain external.
- `data_output`, identity, and ROS parameters cannot change during a run.
- Viewer auto-discovery does not add sensors after RViz2 has started.
- CameraInfo cannot represent a nonzero SDK `k4*r^8` term.
- The latest-frame policy is not suitable for lossless recording.

If no sensor topics appear, verify the sensor stream, IPv4 configuration, UDP
port `7256`, firewall, and interface overrides. If a topic is silent, confirm
that it has a subscriber and the corresponding sensor stream is not `OFF`.
For RViz2 errors, confirm the fixed frame and unique per-sensor child frames.

## Changelog

See the repository [CHANGELOG.md](../CHANGELOG.md) for release history.

## License

This package and its included SDK source are licensed under the MIT License.
See [LICENSE](LICENSE). Third-party ROS 2 and system dependencies retain their
own licenses.
