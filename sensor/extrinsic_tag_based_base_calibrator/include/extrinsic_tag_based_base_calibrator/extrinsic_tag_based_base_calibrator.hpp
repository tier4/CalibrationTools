// Copyright 2023 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__EXTRINSIC_TAG_BASED_BASE_CALIBRATOR_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__EXTRINSIC_TAG_BASED_BASE_CALIBRATOR_HPP_

#include <extrinsic_tag_based_base_calibrator/apriltag_detection.hpp>
#include <extrinsic_tag_based_base_calibrator/calibration_types.hpp>
#include <extrinsic_tag_based_base_calibrator/ceres/calibration_problem.hpp>
#include <extrinsic_tag_based_base_calibrator/scene_types.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <tier4_calibration_msgs/srv/empty.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <tier4_calibration_msgs/srv/files.hpp>
#include <tier4_calibration_msgs/srv/files_with_frame_and_scene_id.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

class ExtrinsicTagBasedBaseCalibrator : public rclcpp::Node
{
public:
  explicit ExtrinsicTagBasedBaseCalibrator(const rclcpp::NodeOptions & options);

protected:
  /*!
   * Callback to calibrate the base_link to sensor kit using the calibration api
   * @param request the calibration request
   * @param response the calibration reponse
   */
  void calibrationRequestCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response,
    const std::string & sensor_kit_frame, const std::string & calibration_sensor_parent_frame,
    const std::string & calibration_sensor_frame);

  /*!
   * Callback method for the image of the calibration cameras
   * @param msg The camera info msg
   */
  void calibrationImageCallback(
    const sensor_msgs::msg::CompressedImage::SharedPtr & msg, const std::string & camera_frame);

  /*!
   * Callback method for camera info of the calibration cameras
   * @param msg The camera info msg
   */
  void cameraInfoCallback(
    const sensor_msgs::msg::CameraInfo::SharedPtr msg, const std::string & camera_frame);

  /*!
   * Callback method for incoming apriltag detections
   * @param detections_msg The apriltag detections
   */
  void apriltagDetectionsCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg,
    const std::string & camera_frame);

  /*!
   * Callback method for incoming lidartag detections
   * @param detections_msg The lidartag detections
   */
  void lidartagDetectionsCallback(
    const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg,
    const std::string & lidar_frame);

  /*!
   * A function to be called periodically that generates rviz markers to visualize the state of the
   * calibration
   */
  void visualizationTimerCallback();

  /*!
   * Generates the next color to be used from a fixed palette
   * @returns the next color to be used
   */
  std_msgs::msg::ColorRGBA getNextColor();

  /*!
   * Attempts to add external camera images to the scene
   * @param request A vector of files to be added as external images
   * @param response Whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool addExternalCameraImagesCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response);

  /*!
   * Attempts to add detections from the calibration sensors to the scene
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool addCalibrationSensorDetectionsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Empty::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Empty::Response> response);

  // Intrinsics realated services
  /*!
   * Attempts to load the external camera intrinsics from a file
   * @param request Vector containing the intrinsics path
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool loadExternalIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  /*!
   * Saves the external camera intrinsics to a file
   * @param request Vector containing the intrinsics path
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool saveExternalIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  /*!
   * Attempts to caibrate the external camera intrinsics from a set of images containing tags
   * @param request Vector containing the path to the images to use for intrinsic calibration
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool calibrateExternalIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  /*!
   * Attempts to load the calibration camera intrinsics from a file
   * @param request Vector containing the intrinsics path
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool loadCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response);

  /*!
   * Saves the calibration camera intrinsics to a file
   * @param request Vector containing the intrinsics path
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool saveCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response);

  /*!
   * Attempts to caibrate the calibratioon intrinsics from a set of images containing tags
   * @param request Vector containing the path to the images to use for intrinsic calibration
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool calibrateCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId::Response> response);

  // Calibration related services
  /*!
   * Processes a scene by obtaining all the images' detections and 3d poses
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool preprocessScenesCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Empty::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Empty::Response> response);

  /*!
   * Calibrate the base link by estimating the 3d pose of all the tags usin BA and then setting the
   * base_link ad the midpoint between the wheel tags
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool calibrationCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Empty::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Empty::Response> response);

  // Calibration related services
  /*!
   * Load a calibration database from a path
   * @param request The path to load the database from
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool loadDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  /*!
   * Save a calibration database to a path
   * @param request The path to load the database from
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool saveDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  rclcpp::TimerBase::SharedPtr visualization_timer_;

  std::unordered_map<
    std::string, rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr>
    image_sub_map_;
  std::unordered_map<std::string, rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr>
    camera_info_sub_map_;
  std::unordered_map<
    std::string, rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr>
    apriltag_detections_sub_map_;
  std::unordered_map<
    std::string, rclcpp::Subscription<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr>
    lidartag_detections_sub_map_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // Calibration API related services
  std::map<std::string, rclcpp::CallbackGroup::SharedPtr> srv_callback_groups_map_;
  std::map<
    std::string, rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr>
    calibration_api_srv_map_;

  // Scene related services
  rclcpp::Service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>::SharedPtr
    add_external_camera_images_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Empty>::SharedPtr
    add_calibration_sensor_detections_to_scene_srv_;

  // Intrinsics realated services
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    load_external_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    save_external_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    calibrate_external_camera_intrinsics_srv_;

  rclcpp::Service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>::SharedPtr
    load_calibration_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>::SharedPtr
    save_calibration_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::FilesWithFrameAndSceneId>::SharedPtr
    calibrate_calibration_camera_intrinsics_srv_;

  // Calibration related services
  rclcpp::Service<tier4_calibration_msgs::srv::Empty>::SharedPtr process_scenes_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Empty>::SharedPtr calibration_srv_;

  // Calibration related services
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr load_database_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr save_database_srv_;

  // Calibration API parameters and variables
  std::string base_frame_;
  // std::unordered_map<std::string, std::string> lidar_sensor_kit_frame_map_; // sensor calibration
  // frame -> topic std::unordered_map<std::string, std::string>
  // calibration_lidar_parent_frame_map_; std::unordered_map<std::string, std::string>
  // camera_sensor_kit_frame_map_; std::unordered_map<std::string, std::string>
  // calibration_camera_parent_frame_map_;
  std::unordered_map<std::string, std::string> sensor_kit_frame_map_;
  std::unordered_map<std::string, std::string> calibration_sensor_parent_frame_map_;
  std::unordered_map<std::string, std::string> calibration_service_names_map_;

  std::mutex mutex_;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;
  bool calibration_done_;
  cv::Affine3d calibrated_main_sensor_to_base_link_pose_;

  // Parameters
  std::vector<std::string> calibration_lidar_frames_vector_;
  std::vector<std::string> calibration_camera_frames_vector_;
  std::vector<std::string> calibration_sensor_frames_vector_;
  std::string main_calibration_sensor_frame_;

  std::unordered_map<std::string, std::string>
    calibration_lidar_detections_topic_map_;  // sensor calibration frame -> topic
  std::unordered_map<std::string, std::string> calibration_image_detections_topic_map_;
  std::unordered_map<std::string, std::string> calibration_camera_info_topic_map_;
  std::unordered_map<std::string, std::string> calibration_image_topic_map_;

  ApriltagDetectorParameters apriltag_detector_parameters_;
  double lidartag_to_apriltag_scale_;

  std::unordered_map<TagType, TagParameters> tag_parameters_map_;
  std::vector<TagParameters> tag_parameters_vector_;
  TagParameters waypoint_tag_parameters_;
  TagParameters wheel_tag_parameters_;
  TagParameters ground_tag_parameters_;

  std::unordered_map<std::string, TagType> tag_family_and_id_to_type_map_;

  int left_wheel_tag_id;
  int right_wheel_tag_id;

  // Intrinsics calibration
  std::string initial_intrinsic_calibration_board_type_;
  bool initial_intrinsic_calibration_tangent_distortion_;
  int initial_intrinsic_calibration_radial_distortion_coeffs_;
  bool initial_intrinsic_calibration_debug_;
  TagParameters initial_intrinsic_calibration_tag_parameters_;

  // Intrinsics calibration : chessboard
  int initial_intrinsic_calibration_board_cols_;
  int initial_intrinsic_calibration_board_rows_;

  // Optimization parameters
  bool ba_optimize_intrinsics_;
  bool ba_share_intrinsics_;
  bool ba_force_shared_ground_plane_;
  double virtual_lidar_f_;

  // Detections
  std::unordered_map<std::string, GroupedApriltagGridDetections> latest_apriltag_detections_map_;
  std::unordered_map<std::string, LidartagDetections> latest_lidartag_detections_map_;
  std::unordered_map<std::string, sensor_msgs::msg::CompressedImage::SharedPtr>
    latest_calibration_camera_images_map_;

  // Scene building parameters
  std::unordered_map<std::string, std::vector<GroupedApriltagGridDetections>>
    scenes_calibration_apriltag_detections_;  // sensor x scene x grouped grid detections
  std::unordered_map<std::string, std::vector<LidartagDetections>>
    scenes_calibration_lidartag_detections_;
  std::vector<std::vector<std::string>>
    scenes_external_camera_images_;  // scene x external camera images
  std::unordered_map<std::string, std::vector<sensor_msgs::msg::CompressedImage::SharedPtr>>
    scenes_calibration_camera_images_;  // scene x sensor name x image file name

  // Calibration & data
  std::shared_ptr<CalibrationData> data_;
  IntrinsicParameters external_camera_intrinsics_;
  std::unordered_map<std::string, IntrinsicParameters> calibration_camera_intrinsics_map_;

  CalibrationProblem calibration_problem_;

  // Visualization
  std::size_t next_color_index_;
  std::vector<std_msgs::msg::ColorRGBA> precomputed_colors_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__EXTRINSIC_TAG_BASED_BASE_CALIBRATOR_HPP_
