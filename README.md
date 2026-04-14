# Lidar Objects Tracker

A ROS 2 package for detecting and tracking moving objects from 2D LiDAR scans.
Part of the [service_robot stack](/README.md).

![Demo](/demo.gif)

## Overview

The pipeline runs on every incoming `LaserScan`:

1. **Scan → Point Cloud** — raw ranges are converted to 2D Cartesian points.
2. **TF transform** — the point cloud is transformed into `target_frame`.
3. **Static background subtraction** *(optional)* — an occupancy grid filters out walls and fixed furniture.
4. **Radius outlier removal** *(optional)* — isolated points with too few neighbours within a given radius are discarded.
5. **DBSCAN clustering** — remaining points are grouped into object candidates.
6. **Centroid extraction** — a 2D centroid is computed per cluster.
7. **Multi-object tracker** — centroids are fed to the selected tracker (LMB or PMBM) which maintains per-object Kalman filters and existence probabilities.
8. **Publish & visualise** — tracked objects are published on `tracked_objects` and rendered in RViz via `tracked_objects_markers`.

## Packages

### `lidar_objects_tracker`

The main ROS 2 node (`objects_tracker_node`).

| Component | File | Description |
|---|---|---|
| `ObjectsTracker` | `lidar_objects_tracker.hpp/.cpp` | ROS 2 node. Orchestrates the full pipeline. |
| `KalmanFilter2D` | `kalman_filter.hpp` | Constant-velocity Kalman filter. State: `[x, y, vx, vy]`. Uses the Joseph form for numerical stability. |
| `StaticBackgroundSubtractor` | `static_background_subtractor.hpp` | World-fixed occupancy grid. Cells above a threshold are treated as static and removed. |
| `TrackerBase` | `tracker_base.hpp` | Abstract interface. Defines `updateTracks()` and `getTracks()`. Both `LMBTracker` and `PMBMTracker` implement it. |
| `LMBTracker` | `lmb_tracker.hpp` | Labelled Multi-Bernoulli tracker. Each track is an independent Bernoulli with an existence probability, updated via gating + Kalman filter. |
| `PMBMTracker` | `pmbm_tracker.hpp` | Poisson Multi-Bernoulli Mixture tracker. Undetected objects are modelled as a PPP; detections spawn Bernoulli tracks updated via nearest-neighbour gating + Kalman filter. |

### `lidar_objects_tracker_msgs`

Custom message definitions.

| Message | Fields | Description |
|---|---|---|
| `TrackedObject` | `header`, `id`, `position` (Pose2D), `velocity` (Vector3) | Single tracked object with 2D pose and velocity. |
| `TrackedObjects` | `objects[]` | Array of `TrackedObject`. Published on `tracked_objects`. |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | sub | Input LiDAR scan. Remapped to `lidar/base/front/scan` by default. |
| `tracked_objects` | `lidar_objects_tracker_msgs/TrackedObjects` | pub | All active tracked objects. |
| `tracked_objects_markers` | `visualization_msgs/MarkerArray` | pub | RViz markers: centroids, bounding boxes, track spheres, velocity arrows, ID labels. |
| `static_background_grid` | `nav_msgs/OccupancyGrid` | pub | Background occupancy grid *(only when `static_bg_subtractor.publish_grid: true`)*. |
| `filtered_pcl` | `sensor_msgs/PointCloud2` | pub | Point cloud after background removal *(only when `publish_filtered_pcl: true`)*. |

## Parameters

### Node

| Parameter | Default | Description |
|---|---|---|
| `target_frame` | `odom` | TF frame in which tracking is performed. |
| `cluster.neighbor_radius` | `0.2` | DBSCAN neighbourhood radius (m). |
| `cluster.min_points` | `6` | Minimum points to form a cluster. |
| `visualize` | `true` | Publish RViz markers. |
| `visualization_period` | `0.1` | Marker publish interval (s). Only used when `visualize: true`. |
| `enable_static_bg_subtraction` | `false` | Enable the static background filter. |
| `publish_filtered_pcl` | `false` | Publish the point cloud after filtering on `filtered_pcl`. |
| `enable_radius_outlier_removal` | `false` | Enable radius outlier removal to discard isolated points. |
| `tracker` | `lmb` | Which tracker to use: `lmb` or `pmbm`. |

### Static Background Subtractor (`static_bg_subtractor.*`)

| Parameter | Default | Description |
|---|---|---|
| `resolution` | `0.1` | Grid cell size (m). Smaller = more precise, more memory. |
| `hit_increment` | `0.007` | Probability increase per scan when a cell is hit. Controls how fast a new static object is learned. |
| `miss_decrement` | `0.015` | Probability decrease per scan when a cell is not hit. Controls how fast a removed object is forgotten. |
| `threshold` | `0.7` | Cells at or above this probability are classified as static background. |
| `inflation_radius` | `1` | Number of cells to inflate around each occupied cell. Every cell within this radius of a static cell is also masked, eliminating flickering at object edges. |
| `grid_half_size` | `20.0` | Half-width/height of the grid (m). The grid is centred on the robot. |
| `publish_grid` | `true` | Publish the occupancy grid on `static_background_grid` for debugging. |

### Radius Outlier Removal (`radius_outlier_removal.*`)

Only declared when `enable_radius_outlier_removal: true`.
Uses Open3D's `RemoveRadiusOutliers` on the 2D point cloud (z = 0) after the background filter.

| Parameter | Default | Description |
|---|---|---|
| `min_points` | `5` | Minimum number of neighbours a point must have within `radius` to be kept. |
| `radius` | `0.3` | Search radius (m). Points with fewer than `min_points` neighbours within this distance are removed. |

### LMB Tracker (`lmb_tracker.*`)

| Parameter | Default | Description |
|---|---|---|
| `max_dt` | `1.0` | Maximum allowed time step (s). Updates with a larger `dt` are skipped. |
| `gate_threshold` | `6.0` | Squared Mahalanobis distance gate (~95 % confidence for χ² with 2 DOF). |
| `birth_existence_prob` | `0.2` | Initial existence probability assigned to a new track. Keep low so multiple detections are required before a track is confirmed. |
| `death_existence_prob` | `0.05` | Tracks with existence probability below this are deleted. |
| `survival_prob` | `0.99` | Probability that an existing object is still present in the next step. |
| `detection_prob` | `0.99` | Probability that an existing object produces a measurement. |

### PMBM Tracker (`pmbm_tracker.*`)

| Parameter | Default | Description |
|---|---|---|
| `max_dt` | `1.0` | Maximum allowed time step (s). Updates with a larger `dt` are skipped. |
| `gate_threshold` | `6.0` | Squared Mahalanobis distance gate (~95 % confidence for χ² with 2 DOF). |
| `survival_prob` | `0.99` | Probability that an existing Bernoulli survives to the next step. |
| `detection_prob` | `0.9` | Probability that an existing object is detected. |
| `birth_intensity` | `0.05` | Initial existence probability assigned to a newly born Bernoulli track. |
| `prune_threshold` | `0.01` | Bernoullis whose existence probability falls below this are removed. |

### Kalman Filter (`kalman_filter.*`)

Shared by both trackers.

| Parameter | Default | Description |
|---|---|---|
| `pos_uncertainty` | `0.2` | Position uncertainty (std, m) for Kalman filter initialisation and measurement noise. |
| `vel_uncertainty` | `0.4` | Velocity uncertainty (std, m/s) for Kalman filter initialisation. |
| `acc_uncertainty` | `1.0` | Acceleration uncertainty (std, m/s²) used as process noise. |

## Usage

```bash
# Build
colcon build --packages-select lidar_objects_tracker lidar_objects_tracker_msgs

# Run node directly
ros2 run lidar_objects_tracker objects_tracker_node

# Launch without a bag
ros2 launch lidar_objects_tracker objects_tracker_launch.py

# Launch with a bag (shuts down when the bag finishes)
ros2 launch lidar_objects_tracker objects_tracker_launch.py bag_path:=/path/to/bagfile.bag

# Launch with a bag and reset RViz time on startup
ros2 launch lidar_objects_tracker objects_tracker_launch.py bag_path:=/path/to/bagfile.bag reset_rviz:=true
```

## TODOs

* Cluster (measurement) confidence (human size, shape, etc.)
* Tests for each component
* Optionally publish static (and dynamic?) scans
* Improve clustering
* Delete leftover bounding box markers
