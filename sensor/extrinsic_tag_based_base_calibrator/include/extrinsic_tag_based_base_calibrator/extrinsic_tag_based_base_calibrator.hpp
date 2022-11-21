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

#include <geometry_msgs/msg/transform_stamped.hpp>
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
  void visualizationTimerCallback();
  std_msgs::msg::ColorRGBA getNextColor();

  bool addSceneCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  bool addExternalCameraImagesCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  bool addLidarDetectionsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  bool addCameraDetectionsCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  bool addCalibrationImagesCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  // Intrinsics realated services
  bool loadExternalIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  bool saveExternalIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  bool calibrateExternalIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  bool loadCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  bool saveCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  bool calibrateCalibrationIntrinsicsCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  // Calibration related services
  bool preprocessScenesCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  bool calibrationCallback(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response);

  // Calibration related services
  bool loadDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  bool saveDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::Files::Request> request,
    std::shared_ptr<tier4_calibration_msgs::srv::Files::Response> response);

  rclcpp::TimerBase::SharedPtr visualization_timer_;
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
  std::string calibration_sensor_type_;

  ApriltagParameters apriltag_parameters_;
  double waypoint_tag_size_;
  double wheel_tag_size_;
  double ground_tag_size_;
  std::vector<int> waypoint_tag_ids_;
  int left_wheel_tag_id_;
  int right_wheel_tag_id_;
  std::vector<int> wheel_tag_ids_;
  std::vector<int> ground_tag_ids_;
  std::vector<int> intrinsic_calibration_tag_ids_;

  std::map<int, double> tag_size_map_;
  std::unordered_set<int> waypoint_tag_ids_set_;
  std::unordered_set<int> wheel_tag_ids_set_;
  std::unordered_set<int> ground_tag_ids_set_;

  // Scene building parameters
  std::vector<std::string> current_external_camera_images_;
  std::vector<std::string> current_calibration_camera_images_;
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
