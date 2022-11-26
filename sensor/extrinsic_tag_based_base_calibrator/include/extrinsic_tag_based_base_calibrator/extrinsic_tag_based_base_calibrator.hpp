// Copyright 2022 Tier IV, Inc.
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

#include <extrinsic_tag_based_base_calibrator/ceres/calibration_problem.hpp>
#include <extrinsic_tag_based_base_calibrator/types.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_srvs/srv/empty.hpp>

#include <apriltag_msgs/msg/april_tag_detection_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <lidartag_msgs/msg/lidar_tag_detection_array.hpp>
#include <tier4_calibration_msgs/srv/files.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <map>
#include <memory>
#include <string>
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
   * Callback method for incoming apriltag detections
   * @param detections_msg The apriltag detections
   */
  void apriltagDetectionsCallback(
    const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr detections_msg);

  /*!
   * Callback method for incoming lidartag detections
   * @param detections_msg The lidartag detections
   */
  void lidartagDetectionsCallback(
    const lidartag_msgs::msg::LidarTagDetectionArray::SharedPtr detections_msg);

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
   * Attempts to add a new scene to the calibration set
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool addSceneCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
   * Attempts to add external camera images to the scene
   * @param request A vector of files to be added as external images
   * @param response Whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool addExternalCameraImagesCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  /*!
   * Attempts to add detections from the calibration lidar to the scene
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool addLidarDetectionsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
   * Attempts to add detections from the calibration camera to the scene
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool addCameraDetectionsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
   * Attempts to add images from the calibration camera to the scene
   * @param request Vector of files from the calibration camera
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool addCalibrationImagesCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

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
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  /*!
   * Saves the calibration camera intrinsics to a file
   * @param request Vector containing the intrinsics path
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool saveCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  /*!
   * Attempts to caibrate the calibratioon intrinsics from a set of images containing tags
   * @param request Vector containing the path to the images to use for intrinsic calibration
   * @param response whether or not the service callback succeeded
   * @returns whether or not the service callback succeeded
   */
  bool calibrateCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  // Calibration related services
  /*!
   * Processes a scene by obtaining all the images' detections and 3d poses
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool preprocessScenesCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  /*!
   * Calibrate the base link by estimating the 3d pose of all the tags usin BA and then setting the
   * base_link ad the midpoint between the wheel tags
   * @param request Empty request
   * @param response Empty response
   * @returns whether or not the service callback succeeded
   */
  bool calibrationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

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
  rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr
    apriltag_detections_sub_;
  rclcpp::Subscription<lidartag_msgs::msg::LidarTagDetectionArray>::SharedPtr
    lidartag_detections_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

  // Scene related services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr add_scene_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr add_external_camera_images_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr add_lidar_detections_to_scene_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr add_camera_detections_to_scene_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr add_calibration_camera_images_srv_;

  // Intrinsics realated services
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    load_external_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    save_external_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    calibrate_external_camera_intrinsics_srv_;

  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    load_calibration_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    save_calibration_camera_intrinsics_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr
    calibrate_calibration_camera_intrinsics_srv_;

  // Calibration related services
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr process_scenes_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calibration_srv_;

  // Calibration related services
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr load_database_srv_;
  rclcpp::Service<tier4_calibration_msgs::srv::Files>::SharedPtr save_database_srv_;

  // Parameters
  std::string calibration_sensor_frame_;
  bool is_lidar_calibration_;

  ApriltagParameters apriltag_parameters_;
  double lidartag_to_apriltag_scale_;
  double waypoint_tag_size_;
  double wheel_tag_size_;
  double ground_tag_size_;
  std::vector<int> waypoint_tag_ids_;
  int left_wheel_tag_id_;
  int right_wheel_tag_id_;
  std::vector<int> wheel_tag_ids_;
  std::vector<int> ground_tag_ids_;

  // Intrinsics calibration
  std::vector<int> intrinsic_calibration_tag_ids_;
  bool initial_intrinsic_calibration_tangent_distortion_;
  int initial_intrinsic_calibration_radial_distortion_coeffs_;

  std::map<int, double> tag_size_map_;
  std::unordered_set<int> waypoint_tag_ids_set_;
  std::unordered_set<int> wheel_tag_ids_set_;
  std::unordered_set<int> ground_tag_ids_set_;

  // Detections
  apriltag_msgs::msg::AprilTagDetectionArray latest_apriltag_detections_msg_;
  lidartag_msgs::msg::LidarTagDetectionArray latest_lidartag_detections_msg_;

  // Scene building parameters
  std::vector<std::string> current_external_camera_images_;
  std::vector<std::string> current_calibration_camera_images_;
  apriltag_msgs::msg::AprilTagDetectionArray current_apriltag_detections_msg_;
  lidartag_msgs::msg::LidarTagDetectionArray current_lidartag_detections_msg_;

  std::vector<apriltag_msgs::msg::AprilTagDetectionArray> scenes_calibration_apriltag_detections_;
  std::vector<lidartag_msgs::msg::LidarTagDetectionArray> scenes_calibration_lidartag_detections_;
  std::vector<std::vector<std::string>> scenes_external_camera_images_;
  std::vector<std::string> scenes_calibration_camera_images_;

  // Calibration & data
  std::shared_ptr<CalibrationData> data_;
  IntrinsicParameters external_camera_intrinsics_;
  IntrinsicParameters calibration_sensor_intrinsics_;

  CalibrationProblem calibration_problem_;

  // Visualization
  std::size_t next_color_index_;
  std::vector<std_msgs::msg::ColorRGBA> precomputed_colors_;
};

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__EXTRINSIC_TAG_BASED_BASE_CALIBRATOR_HPP_
