# tag_based_pnp_calibrator

A tutorial for this calibrator can be found [here](../docs/tutorials/tag_based_pnp_calibrator.md).

## Purpose

The package `tag_based_pnp_calibrator` allows extrinsic calibration between camera and 3d lidar sensors used in autonomous driving and robotics.

Note that the 3d lidar should have a resolution that is high enough to detect the lidartag [1].

## Inner-workings / Algorithms

The `tag_based_pnp_calibrator` utilizes the PnP (Perspective-n-Point) algorithm, a computer vision technique that is used to estimate the transformation between a set of correspondences of 3d and 2d points (in this case the 3d points come from the lidar and the 2d points come from the camera). For the calibration process, both `apriltag_ros` and `lidartag` packages are required, and they are executed automatically by this package's launcher files.

The `apriltag_ros` package detects apriltag markers from an image and outputs the detection results. Conversely, the `lidartag` package detects lidartag markers and outputs its detection results.

The `tag_based_pnp_calibrator` utilizes the detections from both `apriltag_ros` and `lidartag` packages, employing a Kalman filter to monitor these detections. If the detections converge, the calibrator applies the SQPnP [2] algorithm provided by OpenCV to estimate the transformation between the image points from apriltag and the object points from lidartag.

### Diagram

Below, you can see how the algorithm is implemented in the `tag_based_pnp_calibrator` package.

![segment](../docs/images/tag_based_pnp_calibrator/tag_based_pnp_calibrator.jpg)

## ROS Interfaces

### Input

| Name                        | Type                                         | Description                                              |
| --------------------------- | -------------------------------------------- | -------------------------------------------------------- |
| `camera_info`               | `sensor_msgs::msg::CameraInfo`               | Intrinsic parameters of the camera.                      |
| `lidartag/detections_array` | `lidartag_msgs::msg::LidarTagDetectionArray` | Lidartag detections published by `lidartag` package.     |
| `apriltag/detection_array`  | `apriltag_msgs::msg::AprilTagDetectionArray` | AprilTag detections published by `apriltag_ros` package. |

### Output

| Name                   | Type                                             | Description                            |
| ---------------------- | ------------------------------------------------ | -------------------------------------- |
| `filtered_projections` | `visualization_msgs::msg::MarkerArray`           | Calibration markers for visualization. |
| `calibration_points`   | `tier4_calibration_msgs::msg::CalibrationPoints` | Tag points after calibration.          |

### Services

| Name                    | Type                                                  | Description                                                                              |
| ----------------------- | ----------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| `extrinsic_calibration` | `tier4_calibration_msgs::` `srv::ExtrinsicCalibrator` | Generic calibration service. The call is blocked until the calibration process finishes. |

## Parameters

### Core Parameters

| Name                                          | Type                  | Default Value                    | Description                                                                                                                                                                                                                                                                                   |
| --------------------------------------------- | --------------------- | -------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `calib_rate`                                  | `double`              | `10.0`                           | The frequency in Hz at which the calibration callback is invoked.                                                                                                                                                                                                                             |
| `base_frame`                                  | `std::string`         | `base_link`                      | The `base_frame` is used for visualization.                                                                                                                                                                                                                                                   |
| `min_tag_size`                                | `double`              | `0.6`                            | The size of the apriltag in meters (payload).                                                                                                                                                                                                                                                 |
| `max_tag_distance`                            | `double`              | `20.0`                           | Maximum allowed distance in meters from the camera to the tags.                                                                                                                                                                                                                               |
| `max_allowed_homography_error`                | `double`              | `0.5`                            | Apriltag detections are discarded if the homography error is larger than `max_allowed_homography_error`.                                                                                                                                                                                      |
| `use_receive_time`                            | `bool`                | `false`                          | Flag to determine whether to use the receive time instead of the header timestamps.                                                                                                                                                                                                           |
| `use_rectified_image`                         | `bool`                | `true`                           | Flag to determine whether the input images are treated as rectified or not.                                                                                                                                                                                                                   |
| `calibration_crossvalidation_training_ratio`  | `double`              | `0.7`                            | The ratio of data used for training versus validation during the calibration's cross-validation process.                                                                                                                                                                                      |
| `calibration_convergence_min_pairs`           | `int`                 | `9`                              | The minimum number of apriltag and lidartag detection pairs required to consider the calibration process as potentially converged.                                                                                                                                                            |
| `calibration_convergence_min_area_percentage` | `double`              | `0.005`                          | The minimum percentage of the area that needs to be covered by detections.                                                                                                                                                                                                                    |
| `min_pnp_points`                              | `int`                 | `8`                              | The minimum number of points required for the Perspective-n-Point problem used in calibration to solve the pose estimation.                                                                                                                                                                   |
| `min_convergence_time`                        | `double`              | `6.0`                            | Minimum time in seconds required for the active hypotheses to be considered as converged.                                                                                                                                                                                                     |
| `max_no_observation_time`                     | `double`              | `3.0`                            | Hypotheses can timeout when a new observation arrives and the time difference (dt) in seconds exceeds the `max_no_observation_time` threshold for the same tag ID, or when the time difference in seconds between the latest observation and the earlier observation surpasses the threshold. |
| `new_hypothesis_distance`                     | `double`              | `1.5`                            | Distance threshold in meters with respect to converged hypotheses for creating a new hypothesis.                                                                                                                                                                                              |
| `tag_ids`                                     | `std::vector<int>`    | `[0, 1, 2, 3, 4, 5]`             | The list of tag IDs that are used in the calibration process.                                                                                                                                                                                                                                 |
| `tag_sizes`                                   | `std::vector<double>` | `[0.6, 0.6, 0.6, 0.6, 0.6, 0.6]` | Payload sizes of the tags corresponding to the IDs in meters. The size of the vector should be the same as the size of `tag_ids`.                                                                                                                                                             |
| `lidartag_max_convergence_translation`        | `double`              | `0.05`                           | Threshold in meters for the translation component of the Kalman filter's covariance matrix to consider the hypothesis as converged.                                                                                                                                                           |
| `lidartag_max_convergence_translation_dot`    | `double`              | `0.03`                           | Threshold in meters/second for the velocity component of the Kalman filter's covariance matrix to consider the hypothesis as converged.                                                                                                                                                       |
| `lidartag_max_convergence_rotation`           | `double`              | `3.0`                            | Threshold in degrees for the rotation component of the Kalman filter's covariance matrix to consider the hypothesis as converged.                                                                                                                                                             |
| `lidartag_max_convergence_rotation_dot`       | `double`              | `2.5`                            | Threshold in degrees/second for the angular velocity component of the Kalman filter's covariance matrix to consider the hypothesis as converged.                                                                                                                                              |
| `lidartag_new_hypothesis_translation`         | `double`              | `0.1`                            | Translation threshold in meters for generating a new hypothesis in lidartag tracking.                                                                                                                                                                                                         |
| `lidartag_new_hypothesis_rotation`            | `double`              | `15.0`                           | Rotation threshold in degrees for generating a new hypothesis in lidartag tracking.                                                                                                                                                                                                           |
| `lidartag_measurement_noise_translation`      | `double`              | `0.05`                           | Lidartag's measurement noise (translation) in meters used in the Kalman filter's measurement noise covariance matrix (R).                                                                                                                                                                     |
| `lidartag_measurement_noise_rotation`         | `double`              | `5.0`                            | Lidartag's measurement noise (rotation) in degrees used in the Kalman filter's measurement noise covariance matrix (R).                                                                                                                                                                       |
| `lidartag_process_noise_translation`          | `double`              | `0.01`                           | Lidartag's process noise (translation) in meters used in the Kalman filter's process noise covariance matrix (Q).                                                                                                                                                                             |
| `lidartag_process_noise_translation_dot`      | `double`              | `0.001`                          | Lidartag's process noise (translation velocity) in meters/second used in the Kalman filter's process noise covariance matrix (Q).                                                                                                                                                             |
| `lidartag_process_noise_rotation`             | `double`              | `1.0`                            | Lidartag's process noise (rotation) in degrees used in the Kalman filter's process noise covariance matrix (Q).                                                                                                                                                                               |
| `lidartag_process_noise_rotation_dot`         | `double`              | `0.1`                            | Lidartag's process noise (rotation velocity) in degrees/second used in the Kalman filter's process noise covariance matrix (Q).                                                                                                                                                               |
| `apriltag_max_convergence_translation`        | `double`              | `2.0`                            | Maximum threshold in pixels for corners Kalman filter's covariance matrix to consider the hypothesis as converged.                                                                                                                                                                            |
| `apriltag_new_hypothesis_translation`         | `double`              | `20.0`                           | Translation threshold in pixels for generating a new hypothesis in apriltag tracking.                                                                                                                                                                                                         |
| `apriltag_measurement_noise_translation`      | `double`              | `0.2`                            | Apriltag's measurement noise (translation) in meters used in the Kalman filter's measurement noise covariance matrix (R).                                                                                                                                                                     |
| `apriltag_process_noise_translation`          | `double`              | `0.02`                           | Apriltag's process noise (translation) in meters used in the Kalman filter's process noise covariance matrix (Q).                                                                                                                                                                             |

## Requirements

### lidartag

To perform camera-lidar calibration using this tool, it is necessary to prepare lidartags and lidars with intensity measures. To ensure that no objects obstruct the tag detection and to achieve the most stable detection possible, it is highly recommended to also prepare fixed mounts for these tags, as shown below.

Note that the lidartags we used have a size of 0.8 meters (edge size). This means their apriltag payload is 0.6 meters (black frame's edge size). We have also tried with lidartags with smaller sizes (e.g., 0.6 meters), but to use them the user needs to set several parameters by himself.

<p align="center">
    <img src="../docs/images/tag_based_pnp_calibrator/lidartag-mount.jpg"  alt="lidartag-mount" width="500">
</p>

## References

[1] Jiunn-Kai (Bruce) Huang, Shoutian Wang, Maani Ghaffari, and Jessy W. Grizzle, "LiDARTag: A Real-Time Fiducial Tag System for Point Clouds," in IEEE Robotics and Automation Letters. Volume: 6, Issue: 3, July 2021. <!--cSpell:ignore Jiunn,Shoutian,Jessy,Ghaffari,Maani -->

[2] G. Terzakis and M. Lourakis, "A Consistently Fast and Globally Optimal Solution to the Perspective-n-Point Problem" in ECCV 2020. Lecture Notes in Computer Science, vol 12346. <!--cSpell: ignore Terzakis Lourakis -->

## Known issues/limitations

- The tool uses a basic OpenCV camera model for calibration (plumb bomb).
- Only lidars with an intensity channel can be used (lidartag requirement).
- Only traditional lidars with a rotational scan pattern are compatible (other lidars may be compatible but we provide no guarantees).

## Pro tips/recommendations

- During calibration, ensure that the lidar scan covers the tag, similar to the first example shown in the image below. However, if the tag resolution is low, as in the second example, and the lidar still detects the tag, it may still be acceptable but should be avoided when possible. The third example demonstrates a scenario where the lidar scan fails to cover the tag, resulting in the inability to detect the lidartag.

<p align="center">
    <img src="../docs/images/tag_based_pnp_calibrator/lidar_scan_on_tag.jpg"  alt="lidar_scan_on_tag" width="500">
</p>

- It is highly recommended to place the tag perpendicular to the lidar as shown in the following image:

<p align="center">
    <img src="../docs/images/tag_based_pnp_calibrator/tag_position.jpg"  alt="tag_position" width="500">
</p>
