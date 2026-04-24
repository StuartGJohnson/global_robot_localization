# global_robot_localization

ROS 2 Humble `ament_cmake` package for one-shot C++ global localization in a static
occupancy map.

## Targets

- `global_robot_localization_core`: library target for reusable localization components.
- `global_robot_localization_node`: executable ROS 2 node.

Build this from the ROS 2 workspace that owns this package path:

```bash
colcon build --packages-select global_robot_localization
```

## Runtime Interface

Subscriptions:

- `/map` (`nav_msgs/msg/OccupancyGrid`)
- `/scan` (`sensor_msgs/msg/LaserScan`)

Action server:

- `LocalizeInMap` (`global_robot_localization/action/LocalizeInMap`)

Optional publications:

- `/initialpose` (`geometry_msgs/msg/PoseWithCovarianceStamped`)
- `candidate_poses` (`visualization_msgs/msg/MarkerArray`)

Important parameters:

- `num_scans`: number of recent scans to aggregate while the robot is stationary.
- `base_frame`: frame to transform all scan endpoints into before scoring, typically
  `base_link` or `base_footprint`.
- `transform_timeout_sec`: TF lookup timeout for each collected scan.
- `coarse_xy_step`, `coarse_yaw_step_deg`: global search resolution.
- `top_k`: number of coarse candidates to refine.
- `candidate_min_xy_separation`, `candidate_min_yaw_separation_deg`: optional
  diversity filters for preserving distinct coarse global hypotheses.
- `refine_levels`, `refine_xy_step`, `refine_yaw_step_deg`: local refinement controls.
- `scan_stride`, `max_range`, `min_endpoint_count`: lidar endpoint filtering.
- `free_space_weight`, `free_space_sample_step`: optional ray free-space
  consistency term. When enabled, poses are penalized if observed clear laser
  rays pass through occupied map cells.
- `covariance_step_xy`, `covariance_step_yaw_deg`, `covariance_regularization`: finite-difference covariance controls.
