# Lidar Objects Tracker

A ROS 2 package for detecting and tracking moving objects from 2D LiDAR scans.
Part of the [service_robot stack](/README.md).

![Demo](/demo.gif)

## Overview

The pipeline runs on every incoming `LaserScan`:

1. **Scan → Point Cloud** — raw ranges are converted to 2D Cartesian points.
2. **TF transform** — the point cloud is transformed into `target_frame`.
3. **Static background subtraction** *(optional)* — an occupancy grid filters out walls and fixed furniture.
4. **Radius outlier removal** *(optional)* — isolated points with too few neighbours are discarded.
5. **DBSCAN clustering** — remaining points are grouped into object candidates.
6. **Centroid extraction** — a 2D centroid is computed per cluster.
7. **LMB tracker** — centroids are fed to the Labelled Multi-Bernoulli tracker, which maintains per-object Kalman filters and existence probabilities.
8. **Publish & visualise** — confirmed tracked objects are published and rendered in RViz.

## Components

| Component | File | Description |
|---|---|---|
| `ObjectsTracker` | `lidar_objects_tracker.hpp/.cpp` | ROS 2 node. Orchestrates the full pipeline. |
| `KalmanFilter2D` | `kalman_filter.hpp` | Constant-velocity Kalman filter. State: `[x, y, vx, vy]`. |
| `StaticBackgroundSubtractor` | `static_background_subtractor.hpp` | Robot-centric occupancy grid. Cells above a probability threshold are treated as static background and removed. |
| `LMBTracker` | `lmb_tracker.hpp` | Labelled Multi-Bernoulli tracker. Each track is an independent Bernoulli with an existence probability, updated via Mahalanobis gating and a Kalman filter. |

## Topics

| Topic | Type | Direction | Description |
|---|---|---|---|
| `scan` | `sensor_msgs/LaserScan` | sub | Input LiDAR scan. |
| `tracked_objects` | `lidar_objects_tracker_msgs/TrackedObjects` | pub | Confirmed tracked objects. |
| `tracked_objects_markers` | `visualization_msgs/MarkerArray` | pub | RViz markers: centroids, bounding boxes, track spheres, velocity arrows, ID labels. |
| `static_background_grid` | `nav_msgs/OccupancyGrid` | pub | Background occupancy grid *(when `publish_grid: true`)*. |
| `filtered_pcl` | `sensor_msgs/PointCloud2` | pub | Point cloud after filtering *(when `publish_filtered_pcl: true`)*. |

## Parameters

### Node

| Parameter | Description |
|---|---|
| `target_frame` | TF frame in which tracking is performed. |
| `cluster.neighbor_radius` | DBSCAN neighbourhood radius (m). |
| `cluster.min_points` | Minimum points to form a cluster. |
| `visualize` | Publish RViz markers. |
| `enable_static_bg_subtraction` | Enable the static background filter. |
| `publish_filtered_pcl` | Publish the point cloud after filtering. |
| `enable_radius_outlier_removal` | Enable radius outlier removal. |

### Static Background Subtractor (`static_bg_subtractor.*`)

| Parameter | Description |
|---|---|
| `resolution` | Grid cell size (m). |
| `grid_size` | Full width/height of the grid in metres, centred on the robot. |
| `hit_increment` | Probability increase when a cell is hit. Controls how fast a new static object is learned. |
| `miss_decrement` | Probability decrease when a cell is not hit. Controls how fast a removed object is forgotten. |
| `threshold` | Cells at or above this probability are classified as static. |
| `inflation_radius` | Cells within this radius of a static cell are also masked, reducing edge flickering. |
| `initialize_with_first_scan` | Set all hit cells to maximum probability on the first scan to establish the background immediately. |
| `publish_grid` | Publish the occupancy grid for debugging. |

### Radius Outlier Removal (`radius_outlier_removal.*`)

Only active when `enable_radius_outlier_removal: true`.

| Parameter | Description |
|---|---|
| `radius` | Search radius (m). |
| `min_points` | Minimum neighbours within `radius` required to keep a point. |

### LMB Tracker (`lmb_tracker.*`)

| Parameter | Description |
|---|---|
| `max_dt` | Maximum allowed time step (s). Updates exceeding this are skipped. |
| `gate_threshold` | Squared Mahalanobis distance gate. Controls how far a measurement can be from a track to be associated. |
| `birth_existence_prob` | Initial existence probability for a new track. |
| `death_existence_prob` | Tracks below this probability are deleted. |
| `survival_prob` | Probability that an existing object persists to the next step. |
| `detection_prob` | Probability that an existing object produces a measurement. |
| `clutter_intensity` | Expected number of clutter measurements per unit area. Higher values make the tracker less eager to associate measurements. |
| `confirmation_existence_prob` | Existence probability threshold to confirm a track. |
| `merge_distance` | Confirmed tracks closer than this distance are merged (m). Set to `0` to disable. |
| `max_position_variance` | Position covariance is clamped to this value, preventing unbounded growth during missed detections. |

### Kalman Filter (`kalman_filter.*`)

| Parameter | Description |
|---|---|
| `pos_uncertainty` | Position measurement uncertainty (std, m). |
| `vel_uncertainty` | Initial velocity uncertainty (std, m/s). |
| `acc_uncertainty` | Acceleration process noise (std, m/s²). |

## Usage

```bash
# Build
colcon build --packages-select lidar_objects_tracker lidar_objects_tracker_msgs

# Launch
ros2 launch lidar_objects_tracker objects_tracker_launch.py

# Launch with a bag
ros2 launch lidar_objects_tracker objects_tracker_launch.py bag_path:=/path/to/bagfile.mcap
```
