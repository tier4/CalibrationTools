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

#ifndef EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_
#define EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_

#include <Eigen/Dense>
#include <extrinsic_mapping_based_calibrator/calibration_mapper.hpp>
#include <extrinsic_mapping_based_calibrator/lidar_calibrator.hpp>
#include <extrinsic_mapping_based_calibrator/types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_srvs/srv/empty.hpp>

#include <tier4_calibration_msgs/srv/calibration_database.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <tier4_calibration_msgs/srv/frame.hpp>

#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <list>
#include <map>
#include <mutex>
#include <string>
#include <vector>

class ExtrinsicMappingBasedCalibrator : public rclcpp::Node
{
public:
  using PointPublisher = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>;
  using PointSubscription = rclcpp::Subscription<sensor_msgs::msg::PointCloud2>;
  using FrameService = rclcpp::Service<tier4_calibration_msgs::srv::Frame>;

  ExtrinsicMappingBasedCalibrator(const rclcpp::NodeOptions & options);

protected:
  void requestReceivedCallback(
    const std::string & parent_frame, const std::string & calibration_lidar_base_frame,
    const std::string & calibration_lidar_frame,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

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
  tf2_ros::StaticTransformBroadcaster tf_broascaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  OnSetParametersCallbackHandle::SharedPtr set_param_res_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  std::map<std::string, rclcpp::CallbackGroup::SharedPtr> srv_callback_groups_map_;

  std::map<std::string, PointSubscription::SharedPtr> calibration_pointcloud_subs_;
  PointSubscription::SharedPtr mapping_pointcloud_sub_;

  // rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;
  std::map<
    std::string, rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr>
    calibration_api_server_map_;
  rclcpp::Service<tier4_calibration_msgs::srv::Frame>::SharedPtr keyframe_map_server_;
  std::map<std::string, FrameService::SharedPtr> single_lidar_calibration_server_map_;
  std::map<std::string, FrameService::SharedPtr> multiple_lidar_calibration_server_map_;
  rclcpp::Service<tier4_calibration_msgs::srv::CalibrationDatabase>::SharedPtr
    load_database_server_;
  rclcpp::Service<tier4_calibration_msgs::srv::CalibrationDatabase>::SharedPtr
    save_database_server_;

  rclcpp::TimerBase::SharedPtr publisher_timer_;
  rclcpp::TimerBase::SharedPtr data_matching_timer_;

  // Parameters
  MappingParameters::Ptr mapping_parameters_;
  LidarCalibrationParameters::Ptr calibration_parameters_;

  // Calibration API
  std::map<std::string, bool> calibration_status_map_;
  std::map<std::string, Eigen::Matrix4f> calibration_results_map_;
  std::map<std::string, std::string> sensor_kit_frame_map_;              // calibration parent frame
  std::map<std::string, std::string> calibration_lidar_base_frame_map_;  // calibration child frame
  std::mutex service_mutex_;

  // Mapper
  CalibrationMapper::Ptr mapper_;
  MappingData::Ptr mapping_data_;

  // Calibrators
  std::map<std::string, LidarCalibrator::Ptr> lidar_calibrators_;
};

#endif  // EXTRINSIC_MAPPING_BASED_CALIBRATOR_EXTRINSIC_MAPPING_BASED_CALIBRATOR_HPP_
