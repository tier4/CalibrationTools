# mapping_based_calibrator

A tutorial for this calibrator can be found [here](../../docs/tutorials/mapping_based_calibrator.md)

## Purpose

The package `mapping_based_calibrator` performs extrinsic calibration between 3d lidar sensors, as well as the (partial) calibration between a single 3d lidar and `base_link`.

## Inner-workings / Algorithms

### lidar-lidar calibration

The calibrator is designed to estimate the transformation between multiple lidar sensors. It does so by moving the robot/vehicle, creating a trajectory in which lidars observe the same features, and formulating the calibration problem as a pointcloud registration one.

One of the lidars is used to map the trajectory (denoted as the `mapping lidar`), and once a map created with this lidar, the other lidars (denoted as `calibration lidars`) are registered against an augmented `mapping lidar` pointcloud (through the mapping process), which is equivalent to lidar-lidar calibration (between the `mapping lidar` and each of the `calibration lidars` individually).

The calibration process encompasses three main steps: constructing a map with the `mapping lidar` via moving the vehicle, preprocessing and selecting the calibration data, and finally performing lidar-lidar calibration via pointcloud registration.

General notes about the environment, trajectory, and sensors used:

- The environment should contain features appropriate for mapping (e.g., an open space with no walls is inadequate).
- The data from the lidars needs to be synchronized since we pair and interpolate data from different sensors.
- Since lidar scans get distorted with the vehicle's movement, the trajectory followed by the vehicle should be as slow and continuous as possible. Failure to do this has a detrimental impact on the calibration process.
- In addition to mapping, the different lidars must observe common, highly distinctive features to perform pointcloud registration among them. Good examples are objects with a lack of symmetry, and clear 3d shapes (as opposed to 2d objects like walls).
- The mapping lidar is usually chosen as the one with the highest resolution, range, and field of view.
- The resolution and range of the lidars used have a great impact on how or whether this method can be used. We do not make guarantees about any set of combinations, and in most cases, parameters will need to be modified to maintain a good performance.

Note: although this package can perform calibration between the `mapping lidar` and several `calibration lidars`, the documentation will assume only one `calibration lidar` is used. In the presence of multiple `calibration lidars`, the process is done in parallel in an independent fashion.

#### Step 1: Map construction

As mentioned in the previous section, one of the lidars is termed the `mapping lidar` (set in the launchers via the `mapping_pointcloud` argument), and while the robot/vehicle moves, its data is used to construct a map.

The mapping process is implemented via direct pointcloud registration between individual scans of the `mapping lidar` using either NDT[1] or GICP[2] (the algorithm can be set in the launcher). The output of this step is a series of registered pointclouds (raw pointcloud and its pose in the map) dubbed `frames` (or `keyframes`).

However, not all pointclouds coming from the `mapping lidar` are used in the map creation, since there is a chance of data redundancy, which is known to difficult data processing and the registration process itself. For this reason, we consider the following rules when mapping:

- An incoming lidar scan is compared against an aggregated pointcloud of the latest `local_map_num_keyframes` `keyframes`.
- `keyframes` are lidar scans sampled uniformly every `new_keyframe_min_distance` meters.
- Incoming lidars that are not deemed `keyframes`, are saved as `frames` if their distance to the latest accepted `frame` is over `new_frame_min_distance` meters. Otherwise, the incoming scan is discarded.
- If the vehicle stops (and this fact is detected), a special `stopped frame` is saved, since this data is useful for calibration (still data).
- If the trajectory followed by the `frames` is deemed non-continuous (e.g., high accelerations or data loss), the `frame` at which this fact is detected is deemed a `lost frame` and the new incoming scan will not compare against this or previous frames (essentially restarting the mapping process). Note: although in normal mapping applications this is not acceptable, for calibration purposes we only need sequences of registered pointclouds so this is still allowed. However, whenever possible the user should restart the mapping process if he identifies this issue.

#### Step 2: Calibration data preparation

The data required for calibration is created throughout the mapping process and right before the calibration itself. In particular, the mapping and calibration lidar are expected to have different timestamps so they can not be directly registered. Additionally, the mapping process produces a great amount of potential combinations of pointclouds to register, so the data best suited for calibration needs to be chosen.


##### Data synchronization

As explained in the previous section, pointclouds from the `mapping lidar` and `calibration lidar` have different timestamps which makes registration directly unfeasible. To address this problem, whenever a `keyframe` from the `mapping lidar` is generated, the temporally closest `calibration lidar` pointcloud is associated to it, and the pose of the `mapping lidar` pointcloud is interpolated to the stamp of the `calibration lidar` pointcloud using the map (adjacent frames to the `keyframe`).

However, the interpolation is only an approximation and its use induces an interpolation error that can be detrimental to calibration. For this reason, interpolation statistics like the interpolation time, distance, angle, and estimated dynamics are computed.

The output of this step is a list of what we call `calibration frames`, consisting of the `mapping lidar` `keyframe`, the `calibration lidar` pointcloud, the interpolated pose, and the interpolation statistics.

##### Data selection

At this point, we have obtained a series of `calibration frames` that can be used to perform lidar calibration. However, their contents could have little to no useful information (calibration-wise), their data could be compromised due to an incorrect mapping, or their interpolation error could be non-negligible. For these reasons, we select the calibration data using the following criteria:

- All `calibration frames` "close" to `lost frames` are discarded. The term "close" in this context refers to the fact that the `frames` near the `calibration lidar` `keyframe` from the `calibration frame` are used to augment said pointcloud. This step makes sure no invalid data is used (mapping-wise).
- The interpolation statistics are used to discard `calibration frames`. High interpolation times, distances, angles, speed, and acceleration are not accepted (thresholds are set via parameters).
- `calibration frames` have varying levels of "information" in them, and in some cases, that information may not be useful for calibration. To select the frames more suited for calibration information-wise, the following criteria are used:
  - The Principal Component Analysis (PCA) is applied to the `calibration lidar` pointcloud of the `calibration frames`. In this context, the higher the smallest component of PCA is, the more suited a pointcloud is for calibration.
  - Then, the `calibration frames` are sorted in descending order and they are greedily added to the final calibration set until a maximum budget is reached.
  - However, `calibration frames` will be skipped if another one near it has already been added (using distance criteria in the map).


##### Data preprocessing

When doing source-to-target pointcloud registration, all points in the source pointcloud are projected into the target one, and each source point forms a pair with its closest target one. In the case of sparse pointclouds from lidar scans, this causes convergence issues that are very common in the case of algorithms like `ICP` and still cause problems on others like `GICP`.

For this reason, instead of registering the `calibration lidar` points into the `mapping lidar` ones, we first augment the `mapping lidar` pointclouds with their neighbors in the map within a vicinity. This augmented pointcloud has a very high number of points, which makes pointcloud registration intractable. To solve this, we use voxel subsampling before pointcloud registration.

#### Step 3: Pointcloud registration

Lidar-to-lidar calibration is solved implicitly via the pointcloud registration of `calibration lidar` pointclouds into the augmented `mapping lidar` pointclouds. Each pair of pointclouds produces a registered pose, essentially the calibration pose. Among all of these resulting poses, the one that presents a lower overall error (source to target error among all `calibration frames`) is the one chosen as the output calibration result.

However, as registration algorithms are very sensitive to their initial guess and parameters, we use multiple registrators (`ICP`, `GICP`, and `NDT` with different parameters) in a sequential fashion similar to an ensemble, using as the initial guess at every step the best calibration pose so far.

In addition to calibrating using `calibration frame` independently, we also use `Batched ICP`, which allows us to perform ICP using all the `calibration frames` of each lidar simultaneously.

### base-lidar calibration

In addition of lidar-lidar calibration, we can also utilize the map generated by the `mapping lidar` to partially calibrate the transformation between the `mapping lidar` and the `base_link`. This possible if the assumption that the area around of the vehicle forms a plane holds true.

#### Step 1: Map construction

The first step of base-lidar calibration is identical to the [Step 1](#step-1-map-construction) of `lidar-lidar calibration`.

#### Step 2: Extract ground plane from the pointcloud

After constructing the map, and computing the augmented pointcloud from `mapping lidar`, which is identical to the [Step 2](#step-2-calibration-data-preparation), a RANSAC-based plane estimation algorithm is used to extract the ground plane pointcloud and its mathematical model.

#### Step 3: Estimate transformation

To estimate the transformation between the `mapping lidar` and the `base_link`, the tool needs to calculate the transformation between the lidar and the ground pose, as well as the transformation between the ground pose and the `base_link`.

The transformation between the lidar and the ground pose is calculated by utilizing the normal vector and a point on the ground plane, both obtained in the last step. To estimate the transformation between the ground pose and the `base_link`, the tool first determines the initial ground-pose-to-base-link using the initial lidar-to-base-link and lidar-to-ground-pose transformations. Then, the tool projects this initial ground-pose-to-base-link transformation onto the xy plane to estimate the transformation between the ground pose and the `base_link`. The final lidar to `base_link` pose can be obtained by composing the previous poses.


## ROS Interfaces

### Input

| Name                             | Type                                              | Description                                                                                                                         |
| -------------------------------- | ------------------------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------- |
| `calibration_camera_info_topics` | `sensor_msgs::msg::CameraInfo`                    | The intrinsic parameters of calibration cameras. `calibration_camera_info_topics` is provided via parameters (currently not used).  |
| `calibration_image_topics`       | `sensor_msgs::msg::CompressedImage`               | Calibration cameras' image topics. `calibration_image_topics` is provided via parameters (currently not used).                      |
| `mapping_pointcloud`             | `sensor_msgs::msg::PointCloud2`                   | Pointcloud's topic used for mapping. It is recommended to select the lidar that has the highest resolution and best FOV.            |
| `calibration_pointcloud_topics`  | `sensor_msgs::msg::PointCloud2`                   | Pointclouds' topics used for calibrating with the `mapping pointcloud`. `calibration_pointcloud_topics` is provided via parameters. |
| `detected_objects`               | `autoware_perception_msgs::msg::DetectedObjects`  | Messages containing detected objects, used in the filtering procedure.                                                              |
| `predicted_objects`              | `autoware_perception_msgs::msg::PredictedObjects` | Messages that contain predicted object paths and positions, used in the filtering procedure.                                        |

### Output

| Name                              | Type                                   | Description                                                                                       |
| --------------------------------- | -------------------------------------- | ------------------------------------------------------------------------------------------------- |
| `output_map`                      | `sensor_msgs::msg::PointCloud2`        | Output map constructed from the `mapping_pointcloud`.                                             |
| `frame_path`                      | `nav_msgs::msg::Path`                  | The actual path of the `mapping_pointcloud`.                                                      |
| `frame_predicted_path`            | `nav_msgs::msg::Path`                  | The predicted path of the `mapping_pointcloud`.                                                   |
| `keyframe_path`                   | `nav_msgs::msg::Path`                  | The keyframe path of the `mapping_pointcloud`.                                                    |
| `keyframe_markers`                | `visualization_msgs::msg::MarkerArray` | Keyframe markers.                                                                                 |
| `initial_source_aligned_map`      | `sensor_msgs::msg::PointCloud2`        | Initial map from calibration lidars.                                                              |
| `calibrated_source_aligned_map`   | `sensor_msgs::msg::PointCloud2`        | Calibrated map from calibration lidars.                                                           |
| `target_map`                      | `sensor_msgs::msg::PointCloud2`        | Target map from the `mapping lidar`, used for comparing with the `calibrated_source_aligned_map`. |
| `target_markers`                  | `visualization_msgs::msg::MarkerArray` | Markers for the camera calibrator (currently not used).                                           |
| `base_lidar_augmented_pointcloud` | `sensor_msgs::msg::PointCloud2`        | The ground pointcloud extracted from the augmented pointcloud.                                    |
| `ground_pointcloud`               | `sensor_msgs::msg::PointCloud2`        | The ground pointcloud extracted from the calibrated pointcloud.                                   |

### Services

| Name                    | Type                                                  | Description                                                                              |
| ----------------------- | ----------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `extrinsic_calibration` | `tier4_calibration_msgs::` `srv::ExtrinsicCalibrator` | Generic calibration service. The call is blocked until the calibration process finishes. |
| `stop_mapping`          | `std_srvs::srv::Empty`                                | Stops building the map and starts the calibration process.                               |
| `load_database`         | `std_srvs::srv::Empty`                                | Loads lidar and camera calibration frames from the database (for developers).            |
| `save_database`         | `std_srvs::srv::Empty`                                | Saves lidar and camera calibration frames to the database (for developers).              |

## Parameters

### Core Parameters

| Name                                     | Type                  | Default Value  | Description                                                                                                                                                                                    |
| ---------------------------------------- | --------------------- | -------------- | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `calibrate_base_frame`                   | `bool`                | `false`        | Flag to optionally calibrate the base frame. (base_link).                                                                                                                                      |
| `base_frame`                             | `std::string`         |                | The frame name of the base frame used in base-lidar calibration.                                                                                                                               |
| `map_frame`                              | `std::string`         |                | The frame name of the `map`.                                                                                                                                                                   |
| `calibration_camera_optical_link_frames` | `std::vector<string>` |                | The list of frame names for `calibration camera`. (currently not used)                                                                                                                         |
| `calibration_lidar_frames`               | `std::vector<string>` |                | The list of frame names for `calibration lidars`.                                                                                                                                              |
| `calibration_camera_info_topics`         | `std::vector<string>` |                | The list of camera info topics for `calibration camera`. (currently not used)                                                                                                                  |
| `calibration_image_topics`               | `std::vector<string>` |                | The list of camera image topics for `calibration camera`. (currently not used)                                                                                                                 |
| `calibration_pointcloud_topics`          | `std::vector<string>` |                | The list of pointcloud topics for `calibration lidars`.                                                                                                                                        |
| `mapping_lidar_frame`                    | `std::string`         |                | The frame name of the `mapping_lidar`.                                                                                                                                                         |
| `mapping_registrator`                    | `std::string`         | `NDT` / `GICP` | The type of the PCL registration algorithm used for mapping.                                                                                                                                   |
| `mapping_verbose`                        | `bool`                | `false`        | Verbose output flag for mapping.                                                                                                                                                               |
| `use_rosbag`                             | `bool`                | `true`         | Flag to determine if data should be read from a rosbag file.                                                                                                                                   |
| `mapping_max_frames`                     | `int`                 | `500`          | The maximum number of frames to use for mapping. If the number of frames is larger than this value, the mapper stops and the calibration starts.                                               |
| `local_map_num_keyframes`                | `int`                 | `15`           | The number of keyframes stored in the local map.                                                                                                                                               |
| `dense_pointcloud_num_keyframes`         | `int`                 | `10`           | Dense pointcloud for calibration is generated by integrating the pointclouds in the range of [keyframe_id - `dense_pointcloud_num_keyframes`, keyframe_id + `dense_pointcloud_num_keyframes`]. |
| `mapping_min_range`                      | `double`              | `0.5`          | The minimum range in meters of each lidar pointcloud for mapping.                                                                                                                              |
| `mapping_max_range`                      | `double`              | `60.0`         | The maximum range in meters of each lidar pointcloud for mapping.                                                                                                                              |
| `min_mapping_pointcloud_size`            | `int`                 | `10000`        | The minimum size of the pointcloud required for mapping.                                                                                                                                       |
| `min_calibration_pointcloud_size`        | `int`                 | `500`          | The minimum size of the pointcloud that is necessary for estimating transformation.                                                                                                            |
| `mapping_lost_timeout`                   | `double`              | `1.0`          | Sensor's timeout in seconds to consider the mapping process is failed.                                                                                                                         |


### Mapping Parameters

| Name                                 | Type     | Default Value | Description                                                                                                                        |
| ------------------------------------ | -------- | ------------- | ---------------------------------------------------------------------------------------------------------------------------------- |
| `mapper_resolution`                  | `double` | `5.0`         | Resolution for `pclomp::NormalDistributionsTransform` algorithm.                                                                   |
| `mapper_step_size`                   | `double` | `0.1`         | Step size for `pclomp::NormalDistributionsTransform` algorithm.                                                                    |
| `mapper_max_iterations`              | `int`    | `35`          | The maximum number of iterations for `pclomp::NormalDistributionsTransform` and `pcl::GeneralizedIterativeClosestPoint` algorithm. |
| `mapper_epsilon`                     | `double` | `0.01`        | Epsilon value for `pclomp::NormalDistributionsTransform` and `pcl::GeneralizedIterativeClosestPoint` algorithm.                    |
| `mapper_num_threads`                 | `int`    | `8`           | The number of threads to use for `pclomp::NormalDistributionsTransform` algorithm.                                                 |
| `mapper_max_correspondence_distance` | `double` | `0.1`         | Maximum correspondence distance in meters for `pcl::GeneralizedIterativeClosestPoint` algorithm.                                   |
| `mapping_viz_leaf_size`              | `double` | `0.15`        | Leaf size in meters for `pcl::VoxelGrid` to voxelize the mapping pointcloud.                                                       |
| `calibration_viz_leaf_size`          | `double` | `0.15`        | Leaf size in meters for `pcl::VoxelGridTriplets` to voxelize the calibration pointcloud.                                           |
| `leaf_size_input`                    | `double` | `0.1`         | Leaf size in meters for `pcl::VoxelGrid` to voxelize the input pointcloud.                                                         |
| `leaf_size_local_map`                | `double` | `0.1`         | Leaf size in meters for `pcl::VoxelGrid` to voxelize the local map.                                                                |
| `leaf_size_dense_map`                | `double` | `0.05`        | Leaf size in meters for `pcl::VoxelGrid` to voxelize the dense map.                                                                |
| `new_keyframe_min_distance`          | `double` | `1.0`         | Minimum distance in meters between consecutive keyframes.                                                                          |
| `new_frame_min_distance`             | `double` | `0.05`        | Minimum distance in meters of a new frame needs to be apart from the last to be processed.                                         |
| `frame_stopped_distance`             | `double` | `0.02`        | Threshold distance in meters to determine if the frame has stopped moving.                                                         |
| `frames_since_stopped_force_frame`   | `int`    | `5`           | If the number of stopped frames is equal to this value, we set it as `keyframe_and_stop frame`.                                    |
| `calibration_skip_keyframes`         | `int`    | `5`           | The number of initial keyframes that are skipped for calibration.                                                                  |

### Calibration Criteria Parameters

| Name                                         | Type     | Default Value | Description                                                                                                                                   |
| -------------------------------------------- | -------- | ------------- | --------------------------------------------------------------------------------------------------------------------------------------------- |
| `max_allowed_interpolated_time`              | `double` | `0.05`        | Maximum allowable time in seconds for frame interpolated time in standard criteria.                                                           |
| `max_allowed_interpolated_distance`          | `double` | `0.05`        | Maximum allowable distance in meters for frame interpolated distance in standard criteria.                                                    |
| `max_allowed_interpolated_angle`             | `double` | `1.0`         | Maximum allowable angle in degrees for frame interpolated angle in standard criteria.                                                         |
| `max_allowed_interpolated_speed`             | `double` | `3.0`         | Maximum allowable speed in meters/second for frame interpolated speed in standard criteria.                                                   |
| `max_allowed_interpolated_accel`             | `double` | `0.4`         | Maximum allowable acceleration in meters/second^2 for frame interpolated acceleration in standard criteria.                                   |
| `max_allowed_interpolated_distance_straight` | `double` | `0.08`        | Maximum allowable distance in meters for frame interpolated distance in straight criteria.                                                    |
| `max_allowed_interpolated_angle_straight`    | `double` | `0.5`         | Maximum allowable angle in degrees for frame interpolated angle in straight criteria.                                                         |
| `max_allowed_interpolated_speed_straight`    | `double` | `5.0`         | Maximum allowable speed in meters/second for frame interpolated speed in straight criteria.                                                   |
| `max_allowed_interpolated_accel_straight`    | `double` | `0.1`         | Maximum allowable acceleration in meters/second^2 for frame interpolated acceleration in straight criteria.                                   |
| `filter_detections`                          | `bool`   | `true`        | Flag to enable filtering of detection to improve calibration accuracy and reduce noise.                                                       |
| `detection_max_time_tolerance`               | `double` | `0.5`         | Maximum time tolerance in seconds for obtaining all detections close in time to the source pointcloud.                                        |
| `detection_size_tolerance`                   | `double` | `0.4`         | Tolerance for detection size in meters to account for measurement errors and environmental factors.                                           |
| `lost_frame_max_angle_diff`                  | `double` | `25.0`        | Maximum allowable angular difference in degrees between keyframes to consider a frame as lost, used in handling outliers in frame processing. |
| `lost_frame_interpolation_error`             | `double` | `0.05`        | Maximum allowable interpolation error in meters between keyframes to consider a frame as lost, used in handling outliers in frame processing. |
| `lost_frame_max_acceleration`                | `double` | `8.0`         | Maximum allowable acceleration in meters/second between keyframes to consider a frame as lost, used in handling outliers in frame processing. |
| `viz_max_range`                              | `double` | `80.0`        | Maximum range in meters for visualization purposes, setting the boundary for displayable data.                                                |
| `crop_z_calibration_pointclouds`             | `bool`   | `true`        | Flag to enable cropping of the Z-axis in calibration pointclouds.                                                                             |
| `crop_z_calibration_pointclouds_value`       | `double` | `2.0`         | Value of cropping the Z-axis in the calibration pointcloud.                                                                                   |
| `calibration_use_only_stopped`               | `bool`   | `false`       | Flag to use only data from stopped frames.                                                                                                    |
| `calibration_use_only_last_frames`           | `bool`   | `false`       | Flag to use only last frames for calibration.                                                                                                 |
| `max_calibration_range`                      | `double` | `80.0`        | Maximum range in meters to consider for calibration purposes, defining the spatial boundary for calibration data.                             |
| `min_calibration_range`                      | `double` | `1.5`         | Minimum range in meters to consider for calibration purposes, defining the spatial boundary for calibration data.                             |
| `calibration_min_pca_eigenvalue`             | `double` | `0.25`        | If the eigenvalue of a pointcloud is less than this value, it will be filtered out.                                                           |
| `calibration_min_distance_between_frames`    | `double` | `5.0`         | Threshold for the minimum distance in meters between frames.                                                                                  |
| `calibration_eval_max_corr_distance`         | `double` | `0.1`         | Maximum correspondence distance in meters for source pointcloud and target pointcloud.                                                        |

### Calibration Parameters

| Name                       | Type     | Default Value | Description                                                                                                              |
| -------------------------- | -------- | ------------- | ------------------------------------------------------------------------------------------------------------------------ |
| `solver_iterations`        | `int`    | `200`         | Number of iterations for the PCL registration algorithm during calibration, affecting the convergence rate and accuracy. |
| `max_corr_dist_coarse`     | `double` | `0.5`         | Maximum coarse correspondence distance in meters for the PCL registration algorithm during calibration.                  |
| `max_corr_dist_fine`       | `double` | `0.1`         | Maximum fine correspondence distance in meters for the PCL registration algorithm during calibration.                    |
| `max_corr_dist_ultra_fine` | `double` | `0.05`        | Maximum ultra fine correspondence distance in meters for the PCL registration algorithm during calibration.              |

### Lidar Calibration Parameters

| Name                           | Type  | Default Value | Description                                                           |
| ------------------------------ | ----- | ------------- | --------------------------------------------------------------------- |
| `lidar_calibration_min_frames` | `int` | `2`           | The minimum number of calibration frames to use in lidar calibration. |
| `lidar_calibration_max_frames` | `int` | `10`          | The maximum number of calibration frames to use in lidar calibration. |

### Camera Calibration Parameters (currently not used)

| Name                            | Type     | Default Value | Description                                                      |
| ------------------------------- | -------- | ------------- | ---------------------------------------------------------------- |
| `camera_calibration_min_frames` | `int`    | `1`           | The minimum number of frames to consider for camera calibration. |
| `camera_calibration_max_frames` | `int`    | `10`          | The maximum number of frames to use in camera calibration.       |
| `pc_features_min_distance`      | `double` | `0.2`         | Near plane distance in meters for `pcl::FrustumCulling`.         |
| `pc_features_max_distance`      | `double` | `40.0`        | Far plane distance in meters for `pcl::FrustumCulling`.          |

### Base-Lidar Calibration Parameters

| Name                                     | Type     | Default Value | Description                                                                                                     |
| ---------------------------------------- | -------- | ------------- | --------------------------------------------------------------------------------------------------------------- |
| `base_lidar_crop_box_min_x`              | `double` | `-20.0`       | Minimum x-coordinate in meters for the cropping box in base-lidar calibration to focus on relevant data areas.  |
| `base_lidar_crop_box_min_y`              | `double` | `-20.0`       | Minimum y-coordinate in meters for the cropping box in base-lidar calibration to focus on relevant data areas.  |
| `base_lidar_crop_box_min_z`              | `double` | `-20.0`       | Minimum z-coordinate in meters for the cropping box in base-lidar calibration to focus on relevant data areas.  |
| `base_lidar_crop_box_max_x`              | `double` | `20.0`        | Maximum x-coordinate in meters for the cropping box in base-lidar calibration to focus on relevant data areas.  |
| `base_lidar_crop_box_max_y`              | `double` | `20.0`        | Maximum y-coordinate in meters for the cropping box in base-lidar calibration to focus on relevant data areas.  |
| `base_lidar_crop_box_max_z`              | `double` | `20.0`        | Maximum z-coordinate in meters for the cropping box in base-lidar calibration to focus on relevant data areas.  |
| `base_lidar_max_inlier_distance`         | `double` | `0.01`        | Maximum inlier distance in meters for ground extraction by using `pcl::SACSegmentation`.                        |
| `base_lidar_max_iterations`              | `int`    | `1000`        | The maximum number of iterations for ground extraction by using `pcl::SACSegmentation`.                         |
| `base_lidar_min_plane_points`            | `int`    | `1000`        | The minimum number of points required in a pointcloud, ensuring sufficient data for applying ground extraction. |
| `base_lidar_min_plane_points_percentage` | `double` | `10.0`        | The minimum percentage of the ground plane points in a pointcloud.                                              |
| `base_lidar_max_cos_distance`            | `double` | `0.5`         | Maximum cosine distance for applying ground plane extraction.                                                   |
| `base_lidar_overwrite_xy_yaw`            | `bool`   | `false`       | Flag to allow overwriting the x, y, and yaw value during base-lidar calibration.                                |

## Known issues/limitations

- As described in [Step 2](#step-2-calibration-data-preparation), the calibrator interpolates the pose of the `mapping lidar` at the timestamp of the `calibration lidars`. Therefore, the calibrator has strict sensor synchronization requirements.
- It is required that the `mapping lidar` has a high resolution. For the `calibration lidar`, both high and low resolutions are acceptable. Therefore, vehicles with only low-resolution lidars cannot use the calibrator.
- A good initial calibration is required for this calibrator (due to the nature of pointcloud registration).

## Pro tips/recommendations

- It is recommended to select the lidar that has the highest resolution and best FOV as the `mapping lidar`.
- To build the map accurately, drive your vehicle at the lowest feasible speed, such as 2 km/h. Driving too fast can distort the point cloud, negatively impacting the map's accuracy.
- The surroundings of the calibration area are crucial for creating an accurate map and estimating transformations. Therefore, it is essential to ensure that the environment is rich in natural landmarks suitable for registration-based mapping and calibration in all directions, as shown in the image below. This richness in natural landmarks helps the lidar capture sufficient details beyond simple features like lane surfaces or walls, thereby enhancing the accuracy and reliability of the mapping and calibration processes.


<p align="center">
    <img src="../../docs/images/mapping_based_calibrator/mapping_based_vis.svg" alt="radar_reflector" width="900">
<p align="center">

[1] Peter Biber and Wolfgang Straßer, "The normal distributions transform: A new approach to laser scan matching" in Proceedings 2003 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS 2003). <!--cSpell:ignore Biber,Wolfgang,Straßer,IROS -->

[2] Aleksandr V. Segal, Dirk Haehnel, and Sebastian Thrun, "Generalized-ICP" in Robotics: Science and Systems 2009.. <!--cSpell:ignore Aleksandr,Segal,Haehnel,Thrun -->
