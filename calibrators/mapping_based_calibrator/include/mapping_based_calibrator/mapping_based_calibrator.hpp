// Copyright 2024 TIER IV, Inc.
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

#ifndef MAPPING_BASED_CALIBRATOR__MAPPING_BASED_CALIBRATOR_HPP_
#define MAPPING_BASED_CALIBRATOR__MAPPING_BASED_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <mapping_based_calibrator/base_lidar_calibrator.hpp>
#include <mapping_based_calibrator/calibration_mapper.hpp>
#include <mapping_based_calibrator/camera_calibrator.hpp>
#include <mapping_based_calibrator/lidar_calibrator.hpp>
#include <mapping_based_calibrator/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_srvs/srv/empty.hpp>

#include <autoware_perception_msgs/msg/detected_objects.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_calibration_msgs/srv/calibration_database.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <tier4_calibration_msgs/srv/frame.hpp>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

class ExtrinsicMappingBasedCalibrator : public rclcpp::Node
{
public:
  using CameraInfoSubscription = rclcpp::Subscription<sensor_msgs::msg::CameraInfo>;
  using ImageSubscription = rclcpp::Subscription<sensor_msgs::msg::CompressedImage>;
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  explicit ExtrinsicMappingBasedCalibrator(const rclcpp::NodeOptions & options);

protected:
  /*!
   * External interface to start the calibration process and retrieve the result.
   * The call gets blocked until the calibration finishes
   *
   * @param request An empty service request
   * @param response A vector of calibration results
   */
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  /*!
   * Message callback for detected objects
   * @param[in] objects Calibration pointcloud msg
   */
  void detectedObjectsCallback(
    const autoware_perception_msgs::msg::DetectedObjects::SharedPtr objects);

  /*!
   * Message callback for detected objects
   * @param[in] objects Calibration pointcloud msg
   */
  void predictedObjectsCallback(
    const autoware_perception_msgs::msg::PredictedObjects::SharedPtr objects);

  /*!
   * Callback to set parameters using the ROS interface
   * @param[in] parameters vector of new parameters
   */
  rcl_interfaces::msg::SetParametersResult paramCallback(
    const std::vector<rclcpp::Parameter> & parameters);

  void loadDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response);
  void saveDatabaseCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::CalibrationDatabase::Response> response);

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  std::map<std::string, CameraInfoSubscription::SharedPtr> calibration_camera_info_subs_;
  std::map<std::string, ImageSubscription::SharedPtr> calibration_image_subs_;
  std::map<std::string, PointSubscription::SharedPtr> calibration_pointcloud_subs_;
  PointSubscription::SharedPtr mapping_pointcloud_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::DetectedObjects>::SharedPtr
    detected_objects_sub_;
  rclcpp::Subscription<autoware_perception_msgs::msg::PredictedObjects>::SharedPtr
    predicted_objects_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;
  rclcpp::Service<tier4_calibration_msgs::srv::Frame>::SharedPtr keyframe_map_server_;
  std::map<std::string, FrameService::SharedPtr> single_lidar_calibration_server_map_;
  std::map<std::string, FrameService::SharedPtr> multiple_lidar_calibration_server_map_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr base_link_calibration_server_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr stop_mapping_server_;
  rclcpp::Service<tier4_calibration_msgs::srv::CalibrationDatabase>::SharedPtr
    load_database_server_;
  rclcpp::Service<tier4_calibration_msgs::srv::CalibrationDatabase>::SharedPtr
    save_database_server_;

  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::TimerBase::SharedPtr data_matching_timer_;

  // Parameters
  MappingParameters::Ptr mapping_parameters_;
  CalibrationParameters::Ptr calibration_parameters_;

  // Calibration API
  std::map<std::string, bool> calibration_pending_map_;
  std::mutex service_mutex_;

  // Mapper
  CalibrationMapper::Ptr mapper_;
  MappingData::Ptr mapping_data_;

  // Calibrators
  bool calibration_pending_{false};
  std::map<std::string, CameraCalibrator::Ptr> camera_calibrators_;
  std::map<std::string, LidarCalibrator::Ptr> lidar_calibrators_;
  BaseLidarCalibrator::Ptr base_lidar_calibrator_;
};

#endif  // MAPPING_BASED_CALIBRATOR__MAPPING_BASED_CALIBRATOR_HPP_
