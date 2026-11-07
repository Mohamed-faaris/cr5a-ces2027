# Upstream provenance

This repository contains a CR5-focused integration of selected components from:

- Upstream: https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V4
- Source branch: feature/v4-optimization
- Source commit: 0f21e9839888f188ac06b0a9fea622d13a29a74
- ROS distribution: ROS 2 Humble
- Upstream repository license: MIT (see LICENSE)

Imported packages are kept in their upstream package layout. The CR5-only subset of `dobot_rviz` and `cra_description` contains the CR5 URDF and mesh assets; other robot-model assets were intentionally excluded.

The upstream package manifests retain their declared license metadata. Check applicable package-level licensing before redistributing individual components.
