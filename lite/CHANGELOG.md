# Changelog

## [2.0.0] - 2026-07-15

Initial release of `ilidar_lite_ros2`.

### Added

- Multi-device `/ilidar_lite_<SN>` depth, amplitude, intensity, confidence,
  point-cloud, CameraInfo, info, TF, and diagnostics outputs.
- Stable `mono16` depth/amplitude/intensity and binary `mono8` confidence
  contracts across supported sensor encodings.
- Per-SN worker threads, reusable snapshots, subscriber-driven processing, and
  latest-pending-frame replacement.
- Driver-only and supervised RViz2 launch modes with automatic SN discovery.
- Foxy, Humble, and Jazzy build compatibility.
