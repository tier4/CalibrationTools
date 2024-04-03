# tag_based_sfm_calibrator

## Purpose

The package `tag_based_sfm_calibrator` allows extrinsic calibration among most sensors and frames used in autonomous driving and robotics.

In particular it allows the following extrinsic calibrations:

- base_frame (e.g., `base_link`)
- cameras
- lidars (see the compatibility list in Lidartag [1])

Note: depending on how this tool is configured it can perform the following calibrations:

- base-lidar(s)
- base-camera(s)
- camera(s)-lidar(s)
- lidar-lidar(s)
- camera(s)-lidar(s)
- base-camera(s)-lidar(s)

## Inner-workings / Algorithms

Lorem ipsum

## ROS Interfaces

### Input

| Name                                     | Type                                         | Description                                                                                                                                                               |
| ---------------------------------------- | -------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `{calibration_lidar_detections_topics}`  | `lidartag_msgs::msg::LidarTagDetectionArray` | Lidartag detections. `calibration_lidar_detections_topics` is provided via parameters                                                                                     |
| `{calibration_camera_detections_topics}` | `apriltag_msgs::msg::AprilTagDetectionArray` | Apriltag detections. `calibration_camera_detections_topics` is provided via parameters                                                                                    |
| `{calibration_compressed_image_topics}`  | `sensor_msgs::msg::CompressedImage`          | Calibration cameras' image topics. Not used directly for calibration but for debugging and evaluation. `{calibration_compressed_image_topics}` is provided via parameters |
| `{calibration_camera_info_topics}`       | `sensor_msgs::msg::CameraInfo`               | Intrinsic parameters for the calibration cameras . `calibration_camera_info_topics` is provided via parameters                                                            |

### Output

| Name                     | Type                                   | Description         |
| ------------------------ | -------------------------------------- | ------------------- |
| `markers`                | `visualization_msgs::msg::MarkerArray` | Calibration markers |
| `raw_detections_markers` | `visualization_msgs::msg::MarkerArray` | Detection markers   |

### Services

| Name                                             | Type                                               | Description                                                                                                 |
| ------------------------------------------------ | -------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `extrinsic_calibration`                          | `tier4_calibration_msgs::srv::ExtrinsicCalibrator` | Generic calibration service. The call is blocking until the calibration process finishes                    |
| `add_external_camera_images_to_scenes`           | `tier4_calibration_msgs::srv::FilesListSrv`        | Provides a list of external camera images' files for each `scene`                                           |
| `add_calibration_sensor_detections_to_new_scene` | `tier4_calibration_msgs::srv::Empty`               | Created a new `scene` from the latest detections received by the calibrator                                 |
| `load_external_camera_intrinsics`                | `tier4_calibration_msgs::srv::FilesSrv`            | Provides a file containing previously computed external camera intrinsics                                   |
| `save_external_camera_intrinsics`                | `tier4_calibration_msgs::srv::FilesSrv`            | Provides a path so save the computed external camera intrinsics                                             |
| `calibrate_external_camera_intrinsics`           | `tier4_calibration_msgs::srv::FilesSrv`            | Provides a list of files of external camera images to perform intrinsic calibration for the external camera |
| `process_scenes`                                 | `tier4_calibration_msgs::srv::Empty`               | Processed all the obtained `scenes`, mainly applying the tag detector to the external images                |
| `calibrate`                                      | `tier4_calibration_msgs::srv::Empty`               | Uses the processed `scenes` to perform `bundling adjustment` optimization                                   |
| `load_database`                                  | `tier4_calibration_msgs::srv::FilesSrv`            | For debugging purposes. Load a processed database of `scenes`                                               |
| `save_database`                                  | `tier4_calibration_msgs::srv::FilesSrv`            | For debugging purposes. Saves a processed database of `scenes`                                              |

## Parameters

### Core Parameters

| Name                                                     | Type                       | Default Value | Description                                                                                                                                     |
| -------------------------------------------------------- | -------------------------- | ------------- | ----------------------------------------------------------------------------------------------------------------------------------------------- |
| `publish_tfs`                                            | `bool`                     | `N/A`         | Flag to optionally publish the resulting calibration as tfs                                                                                     |
| `write_debug_images`                                     | `bool`                     | `N/A`         | Flag to optionally create images with resulting calibration poses and detections                                                                |
| `base_frame`                                             | `std::string`              | `base_link`   | The `base_frame` is used to compare the initial and calibrated values                                                                           |
| `main_calibration_sensor_frame`                          | `std::string`              | `N/A`         | The sensor whose frame will become the origin during optimization                                                                               |
| `calibration_lidar_frames`                               | `std::vector<std::string>` | `N/A`         | List of the frames corresponding to the calibration lidars                                                                                      |
| `calibration_camera_frames`                              | `std::vector<std::string>` | `N/A`         | List of the frames corresponding to the calibration cameras                                                                                     |
| `lidartag_to_apriltag_scale`                             | double                     |               | The scale factor for converting lidartag detection sizes to apriltag detection sizes                                                            |
| `auxiliar_tag.family`                                    | `std::string`              |               | The family name of the auxiliary tag                                                                                                            |
| `auxiliar_tag.rows`                                      | `int`                      |               | The number of rows in the auxiliary tag                                                                                                         |
| `auxiliar_tag.cols`                                      | `int`                      |               | The number of columns in the auxiliary tag                                                                                                      |
| `auxiliar_tag.size`                                      | `double`                   |               | The size of the auxiliary tag in meters                                                                                                         |
| `auxiliar_tag.spacing`                                   | `double`                   |               | The spacing between auxiliary tags in meters. Only relevant when rows or cols is greater than one                                               |
| `auxiliar_tag.ids`                                       | `std::vector<int64_t>`     |               | The IDs of the auxiliary tags                                                                                                                   |
| `waypoint_tag.family`                                    | `std::string`              |               | The family name of the waypoint tag                                                                                                             |
| `waypoint_tag.rows`                                      | `int`                      |               | The number of rows in the waypoint tag                                                                                                          |
| `waypoint_tag.cols`                                      | `int`                      |               | The number of columns in the waypoint tag                                                                                                       |
| `waypoint_tag.size`                                      | `double`                   |               | The size of the waypoint tag in meters                                                                                                          |
| `waypoint_tag.spacing`                                   | `double`                   |               | The spacing between waypoint tags in meters. Only relevant when rows or cols is greater than one                                                |
| `waypoint_tag.ids`                                       | `std::vector<int64_t>`     |               | The IDs of the waypoint tags                                                                                                                    |
| `ground_tag.family`                                      | `std::string`              |               | The family name of the ground tag                                                                                                               |
| `ground_tag.rows`                                        | `int`                      |               | The number of rows in the ground tag                                                                                                            |
| `ground_tag.cols`                                        | `int`                      |               | The number of columns in the ground tag                                                                                                         |
| `ground_tag.size`                                        | `double`                   |               | The size of the ground tag in meters                                                                                                            |
| `ground_tag.spacing`                                     | `double`                   |               | The spacing between ground tags in meters. Only relevant when rows or cols is greater than one                                                  |
| `ground_tag.ids`                                         | `std::vector<int64_t>`     |               | The IDs of the ground tags                                                                                                                      |
| `wheel_tag.family`                                       | `std::string`              |               | The family name of the wheel tag                                                                                                                |
| `wheel_tag.rows`                                         | `int`                      |               | The number of rows in the wheel tag                                                                                                             |
| `wheel_tag.cols`                                         | `int`                      |               | The number of columns in the wheel tag                                                                                                          |
| `wheel_tag.size`                                         | `double`                   |               | The size of the wheel tag in meters                                                                                                             |
| `wheel_tag.spacing`                                      | `double`                   |               | The spacing between wheel tags in meters. Only relevant when rows or cols is greater than one                                                   |
| `left_wheel_tag_id`                                      | `int`                      |               | The ID of the left wheel tag                                                                                                                    |
| `right_wheel_tag_id`                                     | `int`                      |               | The ID of the right wheel tag                                                                                                                   |
| `ba.optimize_intrinsics`                                 | `bool`                     |               | Flag to optimize the external camera intrinsics during bundle optimization                                                                      |
| `ba.share_intrinsics`                                    | `bool`                     |               | Flag to share intrinsics between different external camera images                                                                               |
| `ba.force_shared_ground_plane`                           | `bool`                     |               | Flag to force the use of a shared ground plane model among the ground tags during bundle optimization                                           |
| `ba.virtual_lidar_f`                                     | `double`                   |               | The focal length of the virtual pinhole model for lidars used in bundle adjustment optimization                                                 |
| `ba.calibration_camera_optimization_weight`              | `double`                   |               | The weight of the camera calibration term in bundle adjustment optimization                                                                     |
| `ba.calibration_lidar_optimization_weight`               | `double`                   |               | The weight of the lidar calibration term in bundle adjustment optimization                                                                      |
| `ba.external_camera_optimization_weight`                 | `double`                   |               | The weight of the external camera calibration term in bundle adjustment optimization                                                            |
| `ba.fixed_ground_plane_model`                            | `bool`                     | false         | Flag to fix the ground plane model during optimization using the values from the initial calibration                                            |
| `initial_intrinsic_calibration.board_type`               | `std::string`              |               | The type of calibration board used for initial intrinsic calibration for the external camera                                                    |
| `initial_intrinsic_calibration.tangent_distortion`       | `bool`                     |               | Flag to enable tangent distortion in initial intrinsic calibration for the external camera                                                      |
| `initial_intrinsic_calibration.radial_distortion_coeffs` | `int`                      |               | The number of radial distortion coefficients used in initial intrinsic calibration for the external camera                                      |
| `initial_intrinsic_calibration.debug`                    | `bool`                     |               | Flag to enable debug mode in initial intrinsic calibration for the external camera                                                              |
| `initial_intrinsic_calibration.tag.family`               | `std::string`              |               | The family name of the tags used in initial intrinsic calibration for the external camera                                                       |
| `initial_intrinsic_calibration.tag.rows`                 | `int`                      |               | The number of rows in the tags used in initial intrinsic calibration for the external camera                                                    |
| `initial_intrinsic_calibration.tag.cols`                 | `int`                      |               | The number of columns in the tags used in initial intrinsic calibration for the external camera                                                 |
| `initial_intrinsic_calibration.tag.size`                 | `double`                   |               | The size of the tags used in initial intrinsic calibration in meters for the external camera                                                    |
| `initial_intrinsic_calibration.tag.spacing`              | `double`                   |               | The spacing between tags used in initial intrinsic calibration in meters for the external camera                                                |
| `initial_intrinsic_calibration.tag.ids`                  | `std::vector<int64_t>`     | [0]           | The IDs of the tags used in initial intrinsic calibration for the external camera                                                               |
| `initial_intrinsic_calibration.board_cols`               | `int`                      |               | The number of columns in the calibration board used for initial intrinsic calibration for the external camera. Only valid for chess-like boards |
| `initial_intrinsic_calibration.board_rows`               | `int`                      |               | The number of rows in the calibration board used for initial intrinsic calibration for the external camera. Only valid for chess-like boards    |
| `apriltag.max_hamming`                                   | `int`                      |               | The maximum allowed Hamming distance for apriltag detection                                                                                     |
| `apriltag.min_margin`                                    | `double`                   |               | The minimum required margin for apriltag detection                                                                                              |
| `apriltag.max_out_of_plane_angle`                        | `double`                   |               | The maximum allowed out-of-plane angle for apriltag detection                                                                                   |
| `apriltag.max_reprojection_error`                        | `double`                   |               | The maximum allowed reprojection error for apriltag detection                                                                                   |
| `apriltag.max_homography_error`                          | `double`                   |               | The maximum allowed homography error for apriltag detection                                                                                     |
| `apriltag.quad_decimate`                                 | `double`                   |               | The decimation factor for quad detection in apriltag detection                                                                                  |
| `apriltag.quad_sigma`                                    | `double`                   |               | The sigma value for quad detection in apriltag detection                                                                                        |
| `apriltag.nthreads`                                      | `int`                      |               | The number of threads to use for apriltag detection                                                                                             |
| `apriltag.debug`                                         | `bool`                     |               | Flag to enable debug mode in apriltag detection                                                                                                 |
| `apriltag.refine_edges`                                  | `bool`                     |               | Flag to enable edge refinement in apriltag detection                                                                                            |

### Lorem ipsum

Lorem ipsum

## Assumptions / Known limits

Lorem ipsum

## Lorem ipsum

Lorem ipsum

## References/External links

[1] Lorem ipsum.

## Known issues / limitations

Lorem ipsum
