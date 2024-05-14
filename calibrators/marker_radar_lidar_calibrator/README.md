# marker_radar_lidar_calibrator

A tutorial for this calibrator can be found [here](../docs/tutorials/marker_radar_lidar_calibrator.md)

## Purpose

The package `marker_radar_lidar_calibrator` allows extrinsic calibration between radar and 3d lidar sensors used in autonomous driving and robotics.

## Inner-workings / Algorithms

The calibrator is designed to accurately predict the transformation between radar and lidar sensors. It starts by pinpointing the central points of reflectors within lidar pointclouds and radar messages, then aligns these points for precise matching. An SVD-based and a yaw-only rotation estimation algorithm are applied to these correlated points to determine the transformation. Specifically, the calibration process consists of four primary steps: constructing a background model, extracting the foreground to detect reflectors, matching and filtering lidar and radar detections, and finally executing the calibration.

### Step 1: Background model construction

Firstly, given the challenge of reliably detecting reflectors, background models for both lidar and radar are constructed from the lidar pointcloud and radar message within a user-defined calibration area, which lacks any calibration targets (such as radar reflectors). More specifically, these background models consist of uniform binary voxel grids that denote whether each voxel represents the background.

### Step 2: Foreground extraction and reflector detection

After the background models for the lidar and radar are established, we extract the foreground points from incoming lidar pointclouds and radar messages that do not align with the background voxels. All foreground radar points are automatically categorized as reflector detections. For foreground lidar points, however, reflector detection involves a more detailed process: we apply a clustering algorithm, perform additional filtering, and calculate the center of each cluster.

### Step 3: Matching and filtering

Since reflector detections cannot be differentiated directly, we rely on the initial calibration to pair each lidar detection with its closest radar detection, and vice versa. A detection pair is accepted if they are mutually the closest matches. Once a match is made, it is evaluated against existing hypotheses: if it aligns with an existing hypothesis, that hypothesis is updated; if it does not align with any, a new hypothesis is created. When a hypothesis achieves convergence, it is finalized and added to the calibration list.

### Step 4: Calibration

Once we have matched detection pairs from the sensors, we can compute the rigid transformation between them using an SVD-based estimation algorithm. Since radar detections lack a Z component, we convert the problem to 2D by setting the Z component of lidar detections to zero, allowing the transformation to be determined in 2d. Currently, we support two algorithms for estimating the transformation: a 2d SVD-based method and a yaw-only rotation estimation. The 2d calibration is generally preferred when valid; otherwise, the yaw rotation is used as the calibration output.

It's also important to note that in the near future, the calibrator will be updated to support radar detections with non-zero Z components using different transformation algorithms.

### Diagram

Below, you can see how the algorithm is implemented in the `marker_radar_lidar_calibrator` package.

![marker_radar_lidar_calibrator](../docs/images/marker_radar_lidar_calibrator/marker_radar_lidar_calibrator.jpg)

## ROS Interfaces

### Input

| Name                     | Type                            | Description                                                                        |
| ------------------------ | ------------------------------- | ---------------------------------------------------------------------------------- |
| `input_lidar_pointcloud` | `sensor_msgs::msg::PointCloud2` | Lidar pointcloud for calibration. `input_lidar_pointcloud` is defined in launcher. |
| `input_radar_msg`        | `radar_msgs::msg::RadarTracks`  | Radar message for calibration, `input_radar_msg` is defined in launcher.           |

### Output

| Name                          | Type                                   | Description                                               |
| ----------------------------- | -------------------------------------- | --------------------------------------------------------- |
| `lidar_background_pointcloud` | `sensor_msgs::msg::PointCloud2`        | Publishes the background pointcloud from lidar.           |
| `lidar_foreground_pointcloud` | `sensor_msgs::msg::PointCloud2`        | Publishes the foreground pointcloud from lidar.           |
| `lidar_colored_clusters`      | `sensor_msgs::msg::PointCloud2`        | Publishes colored pointcloud clusters from lidar.         |
| `lidar_detection_markers`     | `visualization_msgs::msg::MarkerArray` | Publishes lidar detections.                               |
| `radar_background_pointcloud` | `sensor_msgs::msg::PointCloud2`        | Publishes the background pointcloud from radar.           |
| `radar_foreground_pointcloud` | `sensor_msgs::msg::PointCloud2`        | Publishes the foreground pointcloud from radar.           |
| `radar_detection_markers`     | `visualization_msgs::msg::MarkerArray` | Publishes radar detections.                               |
| `matches_markers`             | `visualization_msgs::msg::MarkerArray` | Publishes matched lidar and radar detection.              |
| `tracking_markers`            | `visualization_msgs::msg::MarkerArray` | Publishes tracks of reflectors.                           |
| `text_markers`                | `visualization_msgs::msg::Marker`      | Publishes text markers that show the calibration metrics. |
| `calibration_metrics`         | `std_msgs::msg::Float32MultiArray`     | Publishes calibration metrics.                            |

### Services

| Name                       | Type                                                  | Description                                                                              |
| -------------------------- | ----------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `extrinsic_calibration`    | `tier4_calibration_msgs::` `srv::ExtrinsicCalibrator` | Generic calibration service. The call is blocked until the calibration process finishes. |
| `extract_background_model` | `std_srvs::srv::Empty`                                | Strat to extract the background model from radar and lidar data.                         |
| `add_lidar_radar_pair`     | `std_srvs::srv::Empty`                                | User is able to click this button to add lidar-radar pair.                               |
| `delete_lidar_radar_pair`  | `std_srvs::srv::Empty`                                | User is able to click this button to delete the previous lidar-radar pair.               |
| `send_calibration`         | `std_srvs::srv::Empty`                                | Send the calibration result to the sensor calibration manager.                           |

## Parameters

### Core Parameters

| Name                                        | Type          | Default Value                                           | Description                                                                                                                                       |
| ------------------------------------------- | ------------- | ------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------- |
| `radar_parallel_frame`                      | `std::string` | `base_link`                                             | The frame that the radar frame optimizes the transformation to.                                                                                   |
| `msg_type`                                  | `std::string` | `radar tracks`/`radar scan`                             | The msg type of the input radar message. (Not available yet, currently only support radar tracks)                                                 |
| `transformation_type`                       | `std::string` | `yaw_only_rotation_2d` `svd_2d` `svd_3d` `roll_zero_3d` | The algorithms for optimizing the transformation between radar frame and radar parallel frame. (Not available yet)                                |
| `use_lidar_initial_crop_box_filter`         | `bool`        | `True`                                                  | Enables or disables the initial cropping filter for lidar data processing.                                                                        |
| `lidar_initial_crop_box_min_x`              | `double`      | `-50.0`                                                 | Minimum x-coordinate for the initial lidar calibration area.                                                                                      |
| `lidar_initial_crop_box_min_y`              | `double`      | `-50.0`                                                 | Minimum y-coordinate for the initial lidar calibration area.                                                                                      |
| `lidar_initial_crop_box_min_z`              | `double`      | `-50.0`                                                 | Minimum z-coordinate for the initial lidar calibration area.                                                                                      |
| `lidar_initial_crop_box_max_x`              | `double`      | `50.0`                                                  | Maximum x-coordinate for the initial lidar calibration area.                                                                                      |
| `lidar_initial_crop_box_max_y`              | `double`      | `50.0`                                                  | Maximum y-coordinate for the initial lidar calibration area.                                                                                      |
| `lidar_initial_crop_box_max_z`              | `double`      | `50.0`                                                  | Maximum z-coordinate for the initial lidar calibration area.                                                                                      |
| `use_radar_initial_crop_box_filter`         | `bool`        | `True`                                                  | Enables or disables the initial cropping filter for radar data processing.                                                                        |
| `radar_initial_crop_box_min_x`              | `double`      | `-50.0`                                                 | Minimum x-coordinate for the initial radar calibration area.                                                                                      |
| `radar_initial_crop_box_min_y`              | `double`      | `-50.0`                                                 | Minimum y-coordinate for the initial radar calibration area.                                                                                      |
| `radar_initial_crop_box_min_z`              | `double`      | `-50.0`                                                 | Minimum z-coordinate for the initial radar calibration area.                                                                                      |
| `radar_initial_crop_box_max_x`              | `double`      | `50.0`                                                  | Maximum x-coordinate for the initial radar calibration area.                                                                                      |
| `radar_initial_crop_box_max_y`              | `double`      | `50.0`                                                  | Maximum y-coordinate for the initial radar calibration area.                                                                                      |
| `radar_initial_crop_box_max_z`              | `double`      | `50.0`                                                  | Maximum z-coordinate for the initial radar calibration area.                                                                                      |
| `lidar_background_model_leaf_size`          | `double`      | `0.1`                                                   | Voxel size in meter for the lidar background model.                                                                                               |
| `radar_background_model_leaf_size`          | `double`      | `0.1`                                                   | Voxel size in meter for the radar background model.                                                                                               |
| `max_calibration_range`                     | `double`      | `50.0`                                                  | Maximum range for calibration in meters.                                                                                                          |
| `background_model_timeout`                  | `double`      | `5.0`                                                   | The background model will terminate if it is not updated within this time period, measured in seconds.                                            |
| `min_foreground_distance`                   | `double`      | `0.4`                                                   | Square of this value in meters is used for filtering foreground points, typically needing to be at least double the `background_model_leaf_size`. |
| `background_extraction_timeout`             | `double`      | `15.0`                                                  | Timeout in seconds for background extraction processes.                                                                                           |
| `ransac_threshold`                          | `double`      | `0.2`                                                   | Distance threshold for the segmentation model.                                                                                                    |
| `ransac_max_iterations`                     | `int`         | `100`                                                   | Maximum number of iterations for the segmentation model.                                                                                          |
| `lidar_cluster_max_tolerance`               | `double`      | `0.5`                                                   | Maximum cluster tolerance for extracting lidar cluster.                                                                                           |
| `lidar_cluster_min_points`                  | `int`         | `3`                                                     | Minimum number of points required to form a valid lidar cluster.                                                                                  |
| `lidar_cluster_max_points`                  | `int`         | `2000`                                                  | Maximum number of points allowed in a lidar cluster.                                                                                              |
| `radar_cluster_max_tolerance`               | `double`      | `0.5`                                                   | Maximum cluster tolerance for extracting radar cluster.                                                                                           |
| `radar_cluster_min_points`                  | `int`         | `1`                                                     | Minimum number of points required to form a valid radar cluster.                                                                                  |
| `radar_cluster_max_points`                  | `int`         | `10`                                                    | Maximum number of points allowed in a radar cluster.                                                                                              |
| `reflector_radius`                          | `double`      | `0.1`                                                   | Radius of the reflector in meters.                                                                                                                |
| `reflector_max_height`                      | `double`      | `1.2`                                                   | Maximum height of the reflector in meters.                                                                                                        |
| `max_matching_distance`                     | `double`      | `1.0`                                                   | Maximum distance threshold in meters for matching lidar and radar.                                                                                |
| `max_initial_calibration_translation_error` | `double`      | `1.0`                                                   | Maximum allowable translation error in meters in the calibration process, if it is more than the value, a WARNING will show.                      |
| `max_initial_calibration_rotation_error`    | `double`      | `45.0`                                                  | Maximum allowable rotation error in degree in the calibration process, if it is more than the value, a WARNING will show.                         |
| `max_number_of_combination_samples`         | `int`         | `10000`                                                 | Maximum number of samples from combinations that are used for cross-validation during the calibration process.                                    |

## Requirements

### radar reflector

The type of reflector shown in the image below is crucial for such calibrations because it has a highly predictable and consistent response to radar. The triangular shape, often composed of three metal plates arranged in a prism form, ensures that the reflector returns signals in specific, predictable ways.

<p align="center">
    <img src="../docs/images/marker_radar_lidar_calibrator/radar_reflector.png" alt="radar_reflector" width="150">
<p align="center">

## Known issues/limitations

- While performing the calibration, the calibrator provide a button to delete any mismatched pairs (e.g., an object detected by both radar and lidar). However, some outliers may not be easily detectable by human vision, leading to imperfect results as the calibration proceeds even with these anomalies present. Future enhancements will aim to improve outlier detection, thereby refining the calibration accuracy.

## Pro tips/recommendations

- While performing the calibration, try setting the radar reflector at different heights using the tripod, and also place the reflector at various distances. Please also ensure that the center of the radar reflector faces the radar sensor.
