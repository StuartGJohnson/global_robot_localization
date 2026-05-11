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

## Parameters

The localization node is `global_robot_localization`. Defaults below are the
defaults declared in the node source. `params/tests.yaml` may override them for
tests.

### ROS I/O and Frames

- `base_frame` (`string`, default `base_link`): frame into which `/scan`
  endpoints are transformed before scoring. Common values are `base_link` and
  `base_footprint`.
- `num_scans` (`count`, default `1`): number of recent scans to collect for one
  localization action request. The robot is assumed stationary while these are
  collected.
- `max_scan_buffer_size` (`count`, default `100`): maximum number of received
  scans retained internally.
- `scan_collection_timeout_sec` (`s`, default `3.0`): action-time timeout for
  collecting `num_scans`.
- `transform_timeout_sec` (`s`, default `0.2`): TF lookup timeout for each scan
  frame to `base_frame` transform.
- `publish_initialpose` (`bool`, default `true`): publish the result once on
  `/initialpose` unless suppressed by the action goal.
- `publish_markers` (`bool`, default `true`): publish candidate pose markers
  unless suppressed by the action goal.

### Map Interpretation

- `occupied_threshold` (`OccupancyGrid value`, default `50`): cells with
  `nav_msgs/msg/OccupancyGrid::data >= occupied_threshold` are treated as
  occupied. ROS occupancy grids use `-1` for unknown and `0..100` for occupancy
  probability percent; this is not a `0..255` image pixel threshold.
- `unknown_is_occupied` (`bool`, default `false`): if true, unknown cells
  (`-1`) are treated as occupied when building the distance transform and when
  checking free cells.
- `map_padding_xy` (`m`, default `1.0`): free-space padding added around the map
  before computing the Euclidean distance transform. This smooths EDT queries
  near map edges without expanding the coarse search domain.

### Lidar Filtering and Uncertainty

- `scan_stride` (`beam count step`, default `1`): use every Nth laser beam.
- `max_range` (`m`, default `5.5`): maximum accepted scan range. If set to
  `0.0`, the scan message `range_max` is used instead.
- `min_endpoint_count` (`count`, default `80`): minimum number of valid scan
  endpoints required to run localization.
- `sigma_r` (`m`, default `0.03`): standard deviation of radial lidar range
  error.
- `sigma_theta` (`rad`, default `0.005`): standard deviation of lidar angular
  error.
- `alpha_m` (`unitless`, default `1.0`): multiplier converting map resolution to
  map standard deviation, `sigma_m = alpha_m * map_resolution`.

### Search Strategy

- `coarse_xy_step` (`m`, default `0.1`): XY spacing for the global coarse search.
- `coarse_yaw_step_deg` (`deg`, default `5.0`): yaw spacing for the global
  coarse search.
- `top_k` (`count`, default `40`): number of coarse candidates retained for
  refinement.
- `candidate_min_xy_separation` (`m`, default `0.45`): optional diversity
  spacing between retained coarse candidates in XY.
- `candidate_min_yaw_separation_deg` (`deg`, default `8.0`): optional diversity
  spacing between retained coarse candidates in yaw.
- `refine_levels` (`count`, default `4`): number of local refinement levels.
- `refine_xy_step` (`m`, default `0.15`): initial XY step for local refinement.
- `refine_yaw_step_deg` (`deg`, default `4.0`): initial yaw step for local
  refinement.

### Cost Function

- `off_map_distance` (`m`, default `3.0`): distance returned for EDT queries
  outside the padded EDT domain.
- `free_space_weight` (`unitless`, default `0.4`): weight for the optional
  free-space ray consistency penalty.
- `free_space_sample_step` (`m`, default `0.2`): spacing used when sampling along
  lidar rays for the free-space penalty.

### Covariance Estimation

- `covariance_step_xy` (`m`, default `0.03`): finite-difference step for Hessian
  terms in x and y.
- `covariance_step_yaw_deg` (`deg`, default `1.0`): finite-difference step for
  Hessian terms in yaw.
- `covariance_regularization` (`cost curvature`, default `1.0e-3`): diagonal
  regularization added before inverting the finite-difference Hessian.
- `covariance_scale` (`unitless`, default `1.0`): scale factor applied to the
  final 3x3 covariance over x, y, yaw.

### Simulation Node

The `SimNode` class is intended for tests and does not currently have its own
installed executable. It publishes an asymmetric synthetic map once on `/map`
with transient-local QoS and publishes simulated scans on `/scan`.

- `map_frame` (`string`, default `map`): frame id used for the simulated map.
- `scan_frame` (`string`, default `base_link`): frame id used for the simulated
  laser scan.
- `scan_rate_hz` (`Hz`, default `5.0`): publication rate for `/scan`.
- `robot_x` (`m`, default `2.45`): simulated robot x position in the map frame.
- `robot_y` (`m`, default `2.25`): simulated robot y position in the map frame.
- `robot_yaw` (`rad`, default `0.65`): simulated robot heading in the map frame.
- `beam_count` (`count`, default `91`): number of simulated laser beams.
- `field_of_view` (`rad`, default `2*pi`): angular span of the simulated scan.
- `max_range` (`m`, default `5.5`): maximum simulated lidar range.
- `sigma_r` (`m`, default `0.03`): radial lidar standard deviation used when
  computing per-endpoint variance.
- `sigma_theta` (`rad`, default `0.005`): angular lidar standard deviation used
  when computing per-endpoint variance.
- `alpha_m` (`unitless`, default `1.0`): map-resolution scale used by shared
  search options.
