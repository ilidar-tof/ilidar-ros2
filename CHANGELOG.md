# Changelog

All notable changes to the iLidar ROS 2 package collection are documented in
this file.

## [2.0.1] - 2026-07-22

### Changed

- Changed the iTFS-LITE depth visualization from an RViz2 Camera display to an
  Image display, consistent with the amplitude, intensity, and confidence
  image streams.
- Updated collection and package version metadata to V2.0.1.

## [2.0.0] - 2026-07-15

Initial ROS 2 V2 release.

### Added

- `ilidar_itfs_ros2` and `ilidar_lite_ros2` receive-only packages.
- SN-derived multi-device topics, frames, workers, and configuration overrides.
- Stable ROS image contracts, organized PointCloud2, static TF, transient-local
  sensor info, and diagnostics.
- Driver-only and supervised RViz2 launch modes with automatic SN discovery.
- Foxy, Humble, and Jazzy source compatibility.
- Detailed quick-start, parameter, performance, and troubleshooting guides.

### Notes

- Sensor configuration remains the responsibility of the dedicated product
  tools.
- Runtime sensor identity and capture-layout changes require a driver restart.
