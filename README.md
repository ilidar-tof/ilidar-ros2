# iLidar ROS 2 Packages

ROS 2 C++ packages for the **iTFS** and **iTFS-LITE** product families.

Version: `V2.0.0`

> [!WARNING]
> ROS 2 Foxy has reached end of life. Foxy compatibility is retained for
> existing Ubuntu 20.04 deployments; use Humble or Jazzy for new systems.

> [!NOTE]
> Support for ROS 2 Lyrical Luth on Ubuntu 26.04 LTS is planned.

## Quick Start

Create a ROS 2 workspace and place this repository under its `src` directory:

```bash
mkdir -p ~/ilidar_ros2_ws/src
cd ~/ilidar_ros2_ws/src
git clone https://github.com/ilidar-tof/ilidar-ros2.git

source /opt/ros/jazzy/setup.bash
cd ~/ilidar_ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
```

Launch the viewer for the connected product:

```bash
# iTFS
ros2 launch ilidar_itfs_ros2 viewer.launch.py

# iTFS-LITE
ros2 launch ilidar_lite_ros2 viewer.launch.py
```

Each viewer discovers connected sensors and creates one RViz group per serial
number. Closing RViz also stops the driver included by `viewer.launch.py`.

## Package Overview

|Directory|ROS package|Target|Description|
|:---|:---|:---:|:---|
|`itfs/`|`ilidar_itfs_ros2`|iTFS|Images, calibrated point cloud, TF, diagnostics, and RViz2|
|`lite/`|`ilidar_lite_ros2`|iTFS-LITE|Images, CameraInfo, point cloud, TF, diagnostics, and RViz2|

### Interfaces

|Package|Data outputs|Status output|Info output|
|:---|:---|:---|:---|
|`ilidar_itfs_ros2`|`sensor_msgs/msg/Image`, `sensor_msgs/msg/PointCloud2`|`diagnostic_msgs/msg/DiagnosticArray`|`std_msgs/msg/String`|
|`ilidar_lite_ros2`|`sensor_msgs/msg/Image`, `sensor_msgs/msg/CameraInfo`, `sensor_msgs/msg/PointCloud2`|`diagnostic_msgs/msg/DiagnosticArray`|`std_msgs/msg/String`|

Sensor configuration, firmware, flash, and calibration operations are outside
these receive-side ROS packages. Configure the sensor with its dedicated tool
before starting a driver.

## Dependencies and Platform Support

Dependencies are declared in each package's `package.xml` and can be installed
with `rosdep`. The packages use C++17 and CMake 3.8 or newer. OpenCV, PCL, and
Open3D are not required because visualization uses ROS messages and RViz2.

|Ubuntu|ROS 2|Compiler baseline|Validation|
|:---:|:---:|:---:|:---|
|20.04|Foxy|GCC 9 / C++17|Build verified|
|22.04|Humble|GCC 11 / C++17|Build verified|
|24.04|Jazzy|GCC 13 / C++17|Build and sensor runtime verified|
|26.04|Lyrical Luth|TBD|Planned|

## Build Selected Packages

From the workspace root:

```bash
source /opt/ros/<distro>/setup.bash

colcon build --packages-select ilidar_itfs_ros2
colcon build --packages-select ilidar_lite_ros2
```

Use normal `colcon` options when a specific build type is required:

```bash
colcon build --packages-select ilidar_itfs_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Run and Verify

Source the workspace in every terminal before launching a package:

```bash
source ~/ilidar_ros2_ws/install/setup.bash
```

### iTFS

```bash
# Driver only
ros2 launch ilidar_itfs_ros2 itfs.launch.py

# Driver and RViz2 with automatic sensor discovery
ros2 launch ilidar_itfs_ros2 viewer.launch.py

# Create known RViz groups immediately
ros2 launch ilidar_itfs_ros2 viewer.launch.py sensor_sns:=2405,2406
```

Verify the running driver using the detected serial number:

```bash
ros2 node list
ros2 topic echo /ilidar_2405/info --once
ros2 topic echo /ilidar_2405/depth/image_raw --once --field encoding
```

### iTFS-LITE

```bash
# Driver only
ros2 launch ilidar_lite_ros2 lite.launch.py

# Driver and RViz2 with automatic sensor discovery
ros2 launch ilidar_lite_ros2 viewer.launch.py

# Create known RViz groups immediately
ros2 launch ilidar_lite_ros2 viewer.launch.py sensor_sns:=87,88,101
```

Verify the running driver using the detected serial number:

```bash
ros2 node list
ros2 topic echo /ilidar_lite_87/info --once
ros2 topic echo /ilidar_lite_87/depth/image_raw --once --field encoding
```

The `sensor_sns` argument creates RViz groups but does not filter devices
accepted by the driver. Use `fixed_frame:=<frame>` when configured sensors do
not share the default `base_link`.

## Product Data Differences

|Item|iTFS|iTFS-LITE|
|:---|:---|:---|
|Topic prefix|`/ilidar_<SN>`|`/ilidar_lite_<SN>`|
|Images|Depth, intensity/gray|Depth, amplitude, intensity, confidence|
|Image encodings|Depth/intensity `mono16`|Depth/amplitude/intensity `mono16`, confidence `mono8`|
|Point field|Optional intensity|Optional amplitude|
|CameraInfo|Not published; mapping uses a direction table|Published for depth|
|Special mode|Capture mode 0 publishes gray as intensity|Output depends on configured `data_output`|

Both packages publish organized XYZ in meters, static TF, sensor information,
and diagnostics.

## License

The ROS wrappers and included SDK source are licensed under the MIT License.
Copyright 2022-Present HYBO Inc. See [LICENSE](LICENSE). Third-party ROS 2,
RViz2, and system dependencies retain their own licenses.
