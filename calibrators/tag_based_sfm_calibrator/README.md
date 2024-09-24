# tag_based_sfm_calibrator

A tutorial for this calibrator can be found [here](../../docs/tutorials/tag_based_sfm_calibrator.md).

## Purpose

The `tag_based_sfm_calibrator` package performs extrinsic calibration among most sensors and frames used in autonomous driving and robotics.
In particular, it allows the following extrinsic calibrations:

- base_frame (e.g., `base_link`)
- cameras
- lidars (see the list of supported lidars for Lidartag [1] detection in our [repository](https://github.com/tier4/LiDARTag))

Note: depending on how this tool is configured it can perform the following calibrations:

- base-camera(s)
- base-lidar(s)
- camera-camera(s)
- lidar-lidar(s)
- camera(s)-lidar(s)
- base-camera(s)-lidar(s)

## Inner-workings / Algorithms

As the name of the calibrator suggests, this tool relies upon SfM (Structure from Motion) to find the extrinsics between different sensors. However, since we prioritize calibration accuracy, we do not use natural features and local descriptors, but fiducial systems (in our case apriltags/lidartags). Furthermore, since the sensors are attached to a still vehicle, we employ an additional camera (from now on referred to as the `external camera`), which we use to create a graph and connect the different vehicle's sensors, which we optimize using standard bundling adjustment.
By using a particular sensor as the origin during bundle adjustment, the other sensor's poses correspond to the desired extrinsics.

Base calibration, on the other hand, can not be directly formulated as a sensor calibration problem (since the base is not a sensor !). We instead formulate an indirect approach to find the `base_link` by using its definitions: a frame between the rear axle projected to the ground. We place two tags in each of the rear wheels and then define the `base_link` as the intersection between these two tags projected into the ground.

## ROS Interfaces

### Input

| Name                                     | Type                                            | Description                                                                                                                                                               |
| ---------------------------------------- | ----------------------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `{calibration_lidar_detections_topics}`  | `lidartag_msgs::msg::` `LidarTagDetectionArray` | Lidartag detections. `calibration_lidar_detections_topics` is provided via parameters                                                                                     |
| `{calibration_camera_detections_topics}` | `apriltag_msgs::msg::` `AprilTagDetectionArray` | Apriltag detections. `calibration_camera_detections_topics` is provided via parameters                                                                                    |
| `{calibration_compressed_image_topics}`  | `sensor_msgs::msg::` `CompressedImage`          | Calibration cameras' image topics. Not used directly for calibration but for debugging and evaluation. `{calibration_compressed_image_topics}` is provided via parameters |
| `{calibration_camera_info_topics}`       | `sensor_msgs::msg::` `CameraInfo`               | Intrinsic parameters for the calibration cameras . `calibration_camera_info_topics` is provided via parameters                                                            |

### Output

| Name                     | Type                                   | Description         |
| ------------------------ | -------------------------------------- | ------------------- |
| `markers`                | `visualization_msgs::msg::MarkerArray` | Calibration markers |
| `raw_detections_markers` | `visualization_msgs::msg::MarkerArray` | Detection markers   |

### Services

| Name                                                | Type                                                  | Description                                                                                                 |
| --------------------------------------------------- | ----------------------------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| `extrinsic_calibration`                             | `tier4_calibration_msgs::` `srv::ExtrinsicCalibrator` | Generic calibration service. The call is blocking and only returns when the calibration process finishes    |
| `add_external_camera_` `images_to_scenes`           | `tier4_calibration_msgs::` `srv::FilesListSrv`        | Provides a list of external camera images' files for each `scene`                                           |
| `add_calibration_sensor_` `detections_to_new_scene` | `tier4_calibration_msgs::srv::Empty`                  | Creates a new `scene` from the latest detections received by the node                                       |
| `load_external_camera_intrinsics`                   | `tier4_calibration_msgs::srv::FilesSrv`               | Provides a file containing previously computed external camera intrinsics                                   |
| `save_external_camera_intrinsics`                   | `tier4_calibration_msgs::srv::FilesSrv`               | Provides a path to save the computed external camera intrinsics                                             |
| `calibrate_external_camera_intrinsics`              | `tier4_calibration_msgs::srv::FilesSrv`               | Provides a list of files of external camera images to perform intrinsic calibration for the external camera |
| `process_scenes`                                    | `tier4_calibration_msgs::srv::Empty`                  | Processes all the obtained `scenes`, mainly applying the tag detector to the external images                |
| `calibrate`                                         | `tier4_calibration_msgs::srv::Empty`                  | Uses the processed `scenes` to perform `bundling adjustment` optimization                                   |
| `load_database`                                     | `tier4_calibration_msgs::srv::FilesSrv`               | For debugging purposes. Loads a processed database of `scenes`                                              |
| `save_database`                                     | `tier4_calibration_msgs::srv::FilesSrv`               | For debugging purposes. Saves a processed database of `scenes`                                              |

## Parameters

### Core Parameters

<!--cSpell:ignore nthreads -->

| Name                                                        | Type                       | Default Value | Description                                                                                                                                                        |
| ----------------------------------------------------------- | -------------------------- | ------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------ |
| `publish_tfs`                                               | `bool`                     |               | Flag to optionally publish the resulting calibration as tfs                                                                                                        |
| `write_debug_images`                                        | `bool`                     |               | Flag to optionally create images with the resulting calibration poses and detections                                                                               |
| `base_frame`                                                | `std::string`              | `base_link`   | The `base_frame` is used to compare the initial and calibrated values                                                                                              |
| `main_calibration_sensor_frame`                             | `std::string`              |               | The sensor whose frame will act as the origin during optimization                                                                                                  |
| `calibration_lidar_frames`                                  | `std::vector<std::string>` |               | List of the calibration lidars' frames                                                                                                                             |
| `calibration_camera_frames`                                 | `std::vector<std::string>` |               | List of calibration cameras' frames                                                                                                                                |
| `lidartag_to_apriltag_scale`                                | `double`                   |               | The scale factor for converting lidartag detection sizes to apriltag detection sizes                                                                               |
| `auxiliar_tag.family`                                       | `std::string`              |               | The family name of the auxiliary tags                                                                                                                              |
| `auxiliar_tag.rows`                                         | `int`                      |               | The number of rows in the auxiliary tags                                                                                                                           |
| `auxiliar_tag.cols`                                         | `int`                      |               | The number of columns in the auxiliary tags                                                                                                                        |
| `auxiliar_tag.size`                                         | `double`                   |               | The size of the auxiliary tags in meters                                                                                                                           |
| `auxiliar_tag.spacing`                                      | `double`                   |               | The spacing between auxiliary tags (as a proportion relative to the tag size). Only relevant when rows or cols is greater than one                                 |
| `auxiliar_tag.ids`                                          | `std::vector<int64_t>`     |               | The auxiliar tag IDs                                                                                                                                               |
| `waypoint_tag.family`                                       | `std::string`              |               | The family name of the waypoint tag                                                                                                                                |
| `waypoint_tag.rows`                                         | `int`                      |               | The number of rows in the waypoint tag                                                                                                                             |
| `waypoint_tag.cols`                                         | `int`                      |               | The number of columns in the waypoint tag                                                                                                                          |
| `waypoint_tag.size`                                         | `double`                   |               | The size of the waypoint tag in meters                                                                                                                             |
| `waypoint_tag.spacing`                                      | `double`                   |               | The spacing between waypoint tags (as a proportion relative to the tag size). Only relevant when rows or cols is greater than one                                  |
| `waypoint_tag.ids`                                          | `std::vector<int64_t>`     |               | The waypoint tag IDs                                                                                                                                               |
| `ground_tag.family`                                         | `std::string`              |               | The family name of the ground tag                                                                                                                                  |
| `ground_tag.rows`                                           | `int`                      |               | The number of rows in the ground tag                                                                                                                               |
| `ground_tag.cols`                                           | `int`                      |               | The number of columns in the ground tag                                                                                                                            |
| `ground_tag.size`                                           | `double`                   |               | The size of the ground tag in meters                                                                                                                               |
| `ground_tag.spacing`                                        | `double`                   |               | The spacing between ground tags (as a proportion relative to the tag size). Only relevant when rows or cols is greater than one                                    |
| `ground_tag.ids`                                            | `std::vector<int64_t>`     |               | The ground tags IDs                                                                                                                                                |
| `wheel_tag.family`                                          | `std::string`              |               | The family name of the wheel tag                                                                                                                                   |
| `wheel_tag.rows`                                            | `int`                      |               | The number of rows in the wheel tag                                                                                                                                |
| `wheel_tag.cols`                                            | `int`                      |               | The number of columns in the wheel tag                                                                                                                             |
| `wheel_tag.size`                                            | `double`                   |               | The size of the wheel tag in meters                                                                                                                                |
| `wheel_tag.spacing`                                         | `double`                   |               | The spacing between wheel tags (as a proportion relative to the tag size). Only relevant when rows or cols is greater than one                                     |
| `left_wheel_tag_id`                                         | `int`                      |               | The left wheel tag ID                                                                                                                                              |
| `right_wheel_tag_id`                                        | `int`                      |               | The right wheel tag ID                                                                                                                                             |
| `ba.optimize_intrinsics`                                    | `bool`                     |               | Flag to optimize the external camera intrinsics during bundle optimization                                                                                         |
| `ba.share_intrinsics`                                       | `bool`                     |               | Flag to share intrinsics between different external camera images                                                                                                  |
| `ba.force_shared_ground_plane`                              | `bool`                     |               | Flag to force the use of a shared ground plane model among the ground tags during bundle optimization                                                              |
| `ba.virtual_lidar_f`                                        | `double`                   |               | The focal length of the virtual pinhole model for lidars used in the bundle adjustment optimization                                                                |
| `ba.calibration_camera_` `optimization_weight`              | `double`                   |               | The weight of the camera calibration term in the bundle adjustment optimization                                                                                    |
| `ba.calibration_lidar_` `optimization_weight`               | `double`                   |               | The weight of the lidar calibration term in the bundle adjustment optimization                                                                                     |
| `ba.external_camera_` `optimization_weight`                 | `double`                   |               | The weight of the external camera calibration term in the bundle adjustment optimization                                                                           |
| `ba.fixed_ground_plane_model`                               | `bool`                     | `false`       | Flag to fix the ground plane model during the optimization using the values from the initial calibration                                                           |
| `initial_intrinsic_calibration.` `board_type`               | `std::string`              |               | The type of calibration board used for the initial intrinsic calibration for the external camera. Valid values are `apriltag` and `chessboard`                     |
| `initial_intrinsic_calibration.` `tangent_distortion`       | `bool`                     |               | Flag to enable tangent distortion in the initial intrinsic calibration for the external camera                                                                     |
| `initial_intrinsic_calibration.` `radial_distortion_coeffs` | `int`                      |               | The number of radial distortion coefficients used in the initial intrinsic calibration for the external camera                                                     |
| `initial_intrinsic_` `calibration.debug`                    | `bool`                     |               | Flag to enable debug mode in the initial the intrinsic calibration for the external camera                                                                         |
| `initial_intrinsic_` `calibration.tag.family`               | `std::string`              |               | The family name of the tags used in the initial intrinsic calibration for the external camera                                                                      |
| `initial_intrinsic_` `calibration.` `tag.rows`              | `int`                      |               | The number of rows in the tags used in the initial intrinsic calibration for the external camera                                                                   |
| `initial_intrinsic_` `calibration.tag.cols`                 | `int`                      |               | The number of columns in the tags used in the initial intrinsic calibration for the external camera                                                                |
| `initial_intrinsic_` `calibration.tag.size`                 | `double`                   |               | The size of the tags used in the initial intrinsic calibration in meters for the external camera                                                                   |
| `initial_intrinsic_` `calibration.tag.spacing`              | `double`                   |               | The spacing between tags used in the initial intrinsic calibration in meters for the external camera                                                               |
| `initial_intrinsic_` `calibration.tag.ids`                  | `std::vector<int64_t>`     | `[0]`         | The tag IDs used in the initial intrinsic calibration for the external camera. Only used when `board_type` equals `apriltag`                                       |
| `initial_intrinsic_` `calibration.board_cols`               | `int`                      |               | The number of columns in the calibration board used for the initial intrinsic calibration for the external camera. Only used when `board_type` equals `chessboard` |
| `initial_intrinsic_` `calibration.board_rows`               | `int`                      |               | The number of rows in the calibration board used for the initial intrinsic calibration for the external camera. Only used when `board_type` equals `chessboard`    |
| `apriltag.max_hamming`                                      | `int`                      |               | The maximum allowed Hamming distance ins bits for apriltag detection                                                                                               |
| `apriltag.min_margin`                                       | `double`                   |               | The minimum required margin in pixels for apriltag detection                                                                                                       |
| `apriltag.max_out_of_plane_angle`                           | `double`                   |               | The maximum allowed out-of-plane angle in degrees for apriltag detection                                                                                           |
| `apriltag.max_reprojection_error`                           | `double`                   |               | The maximum allowed reprojection error in pixels for apriltag detection                                                                                            |
| `apriltag.max_homography_error`                             | `double`                   |               | The maximum allowed homography error for apriltag detection                                                                                                        |
| `apriltag.quad_decimate`                                    | `double`                   |               | The decimation factor for quad detection in apriltag detection                                                                                                     |
| `apriltag.quad_sigma`                                       | `double`                   |               | The sigma value for quad detection in apriltag detection                                                                                                           |
| `apriltag.nthreads`                                         | `int`                      |               | The number of threads to use for apriltag detection                                                                                                                |
| `apriltag.debug`                                            | `bool`                     |               | Flag to enable debug mode in apriltag detection                                                                                                                    |
| `apriltag.refine_edges`                                     | `bool`                     |               | Flag to enable edge refinement in apriltag detection                                                                                                               |

## Requirements

The following figure presents the elements involved in the calibration process:

![segment](../../docs/images/tag_based_sfm_calibrator/bev_setup.svg)

### Waypoint tag

Waypoint tags are meant to be detected by both lidars and cameras.
Since lidar can not detect the tags if they are in the same plane as other elements, an additional structure is needed to allow the detector algorithm to segment it from other objects (see Figure 2).

Considerations:

- Among all the tags in the environment, waypoints are the only ones that can be moved during experiments (please read the provided tutorial and the concept of scenes).
- The orientation of the waypoint tags should be so that the line that connects the calibration sensors and the waypoint tag is perpendicular to the waypoint tag plane. This is recommended since lidartag detection presents worse performance when this is not the case (more than the detector, it is a limitation of most lidars).
- We so far have used 800mmx800mm (complete side lengths, as opposed to the inner apriltag payload) waypoints, and have worked well for most lidars/configurations in our projects.
  There should be at least one waypoint tag no matter the desired calibration type. However, in practice, the more waypoint tags (tags that the calibration sensors can detect) the faster the calibration process becomes.

### Wheel tag

Wheel tags are what allow us to find the `base_link` via solving the bundle adjustment problem.

Considerations:

- They usually can not be detected by the calibration sensors (instead they are detected by the external camera).
- Since the number of images that contain wheel tags is expected to be low, it is convenient to use grids of individual tags to increase the number of features (tag corners) corresponding to the wheels. An example of a 2x2 tag is presented in Figure 3.
- Since the wheel tags determine the base link, their center should coincide with the axle as much as possible (at least in the x-axis seen from the `base_link`).

### Ground tag

Ground tags are essential during the calibration of the `base_link` since it defines the plane that the `base_link` lives in (an example is presented in Figure 4).

Considerations:

- The surface (ground) in which the vehicle and the ground tags are placed needs to be as flat as possible (plane).
- For this particular tag, the width of the tag is relevant. If it can not be assumed to be null, the final `base_link` pose needs to be adjusted manually manually. E.g., if the pose from `base_link` to lidar is `z=1m` and the tag width is 1cm, the real pose would be `z=1.01m`.
- The ground tags can be printed using normal paper and then attached via tape to the floor. The user needs to be careful not to move them during the experiment (in particular since it is very easy to step on them).
- The ground tags can be laminated for reuse, but the reflections caused by it can make it difficult or impossible to perform detection.

### Auxiliary tag

Auxiliary tags are tags that do not have restrictions on their placement and use and were added to improve the observation distribution in the bundle adjustment graph. If it were not for the auxiliary tags, most of the tags would be on a single plane, save a few observations corresponding to the waypoints and the wheel tags.

Considerations:

- It is recommended to place them in positions and orientations without sufficient samples otherwise. Good candidates are walls, for which an example is presented in Figure 5.

### External camera

The role of the external camera is to connect the sensors and their detections creating the graph required for bundle adjustment.

Considerations:

- The camera must behave like a pinhole + distortion model. That is to say, the lenses must be "fixed" during the experiment.
- The highest "real" resolution the better. So far we have tried with Nikon DSLRs (half frame) with moving and fixed lenses, and a simple point-and-shot camera. <!--cSpell:ignore DSLRs -->
- In the [pro-tips / recommendations](#pro-tipsrecommendations) section, more suggestions regarding the camera settings are provided.

### Initial (external) camera intrinsics calibration board

To use the external camera in bundle adjustment, it is necessary to know, to a certain degree, its intrinsics.
Any intrinsic calibration board that can be used for this purpose is acceptable, but circle-patterned chess boards and apriltags are supported directly by this tool. If you do not have these boards, you will need to calibrate your camera independently and provide directly the camera intrinsics.

<table>
  <tr>
    <td><img src="../../docs/images/tag_based_sfm_calibrator/waypoint_tag.jpg" alt="waypoint" height = 256px ></td>
    <td><img src="../../docs/images/tag_based_sfm_calibrator/wheel_tag.jpg" alt="wheel" width = 256px height = 256px></td>
    <td><img src="../../docs/images/tag_based_sfm_calibrator/ground_tag.jpg" alt="ground" width = 256px height = 256px ></td>
    <td><img src="../../docs/images/tag_based_sfm_calibrator/auxiliar_tag.jpg" alt="auxiliary"width = 256px height = 256px></td>
   </tr>
   <tr>
    <td><p style="text-align: center;">Figure 2. Waypoint tag</p></td>
    <td><p style="text-align: center;">Figure 3. Wheel tag</p></td>
    <td><p style="text-align: center;">Figure 4. Ground tag</p></td>
    <td><p style="text-align: center;">Figure 5. Auxiliar tag</p></td>
  </tr>
</table>

## Calibration process

A complete example with real data is provided in our [tutorial](../../docs/tutorials/tag_based_sfm_calibrator.md), so we recommend users interested in this tools to get hands-on-experience directly with that example. Nevertheless, the calibration process consists of the following steps:

- External camera intrinsic calibration. This can be done directly throught this tool if you have either apriltags or dot boards.
- Place waypoint tags in positions where the calibrations can detect them accurately.
- Obtain detections with the calibrations sensors (through the UI).
- Using the external camera, obtain samples of the environment, taking care to avoid motion blur. The user should focus on connecting the calibrator sensors by taking photos containing multiple waypoints, ot at least photos that contain one waypoints and multiple other tags.
- Repeat N scenes (this depend on the users criteria and availability of waypoints).
- Process the scenes (done through the UI).
- Start the calibration process /graph optimization (done through the UI).

## Known issues/limitations

- Our version of lidartag only supports the family `16h5`.
- Our codebase only supports apriltag detections for `16h5` and `36h11`.
- Ground tags are assumed to have no width. If that is not the case, you can directly compensate the width in the resulting extrinsics (basically just add the width to the z-component of the extrinsics).
- We only use 2 wheel tags. The quality of the calibration could improve, mainly in the yaw component, if we were to use tags in the four wheels.

## Pro tips/recommendations

- If the `base_link` seems flipped in yaw by 180 degrees, it is probably because you mistook the left and right wheel tags.
- Regarding the operation of the external camera:
  - For an optimal depth of field, use the highest f-number possible of the camera and focus the lens to infinity.
  - Prefer fixed lenses and do not forget to turn off auto zoom.
  - One of the biggest concerns is motion blur when using the external camera. Since a high f-number is recommended, instead of increasing the exposure, prefer using a higher ISO or just under-exposed images.
  - Images taken from the external cameras should have as many possible detections in each image. The absolute minimum is 2, but under 3 are discarded.
  - Detections from the external camera should, whenever possible, have the least amount of out-of-plane rotation. This is due to how the pose estimation gets distorted in extreme cases.
  - The sampling of the scene by the external camera should focus on the waypoints and wheel tags whenever possible. In terms of usefulness for calibration, images that contain both waypoints and wheel tags are highly coveted. This is due to how they form the closest possible connection between the calibration sensors and the wheels.

## References/External links

[1] Jiunn-Kai (Bruce) Huang, Shoutian Wang, Maani Ghaffari, and Jessy W. Grizzle, "LiDARTag: A Real-Time Fiducial Tag System for Point Clouds," in IEEE Robotics and Automation Letters. Volume: 6, Issue: 3, July 2021. <!--cSpell:ignore Jiunn,Shoutian,Jessy,Ghaffari,Maani -->
