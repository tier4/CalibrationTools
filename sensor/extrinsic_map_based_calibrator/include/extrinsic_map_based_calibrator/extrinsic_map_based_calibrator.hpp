// Copyright 2021 Tier IV, Inc.
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

#ifndef EXTRINSIC_MAP_BASED_CALIBRATOR__EXTRINSIC_MAP_BASED_CALIBRATOR_HPP_
#define EXTRINSIC_MAP_BASED_CALIBRATOR__EXTRINSIC_MAP_BASED_CALIBRATOR_HPP_

#include <string>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "rclcpp/clock.hpp"

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/registration/gicp.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif
#include "tier4_calibration_msgs/srv/extrinsic_calibrator.hpp"
#include "extrinsic_map_based_calibrator/extrinsic_map_based_preprocessing.hpp"
#include "extrinsic_map_based_calibrator/grid_search_matching.hpp"


namespace extrinsic_map_base_calibrator
{

class ExtrinsicMapBasedCalibrator : public rclcpp::Node
{
private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_with_wall_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_without_wall_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_pointcloud_sub_;
  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr server_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_without_wall_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensor_pointcloud_without_wall_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr calibrated_pointcloud_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::mutex mutex_;
  std::string parent_frame_;
  std::string child_frame_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_with_wall_pointcloud_msg_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr map_without_wall_pointcloud_msg_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr sensor_pointcloud_msg_;
  sensor_msgs::msg::PointCloud2 calibrated_pointcloud_msg_;

  bool is_debug_pub_ = true;

  matchingResult calibrated_sensor_result_;

  ExtrinsicMapBasedPreprocessing preprocessing_;
  GridSearchMatching grid_search_matching_;

public:
  explicit ExtrinsicMapBasedCalibrator(const rclcpp::NodeOptions & node_options);
  bool mapBasedCalibration(const tf2::Transform & tf_initial_pose);
  bool preprocessing(
    PointCloudT::Ptr & pcl_map,
    PointCloudT::Ptr & pcl_map_without_wall,
    PointCloudT::Ptr & pcl_sensor, const tf2::Transform & tf_initial_pose);
  bool convertFromROSMsg(
    PointCloudT::Ptr & pcl_map,
    PointCloudT::Ptr & pcl_map_without_wall,
    PointCloudT::Ptr & pcl_sensor);
  void targetMapWithWallCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void targetMapWithoutWallCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void sourcePointcloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
  void publishPointCloud(
    const PointCloudT::Ptr & pcl_map,
    const PointCloudT::Ptr & pcl_map_without_wall,
    const PointCloudT::Ptr & pcl_sensor,
    const PointCloudT::Ptr & pcl_sensor_without_wall,
    const PointCloudT::Ptr & pcl_result);
  void publishPointCloud(const PointCloudT::Ptr & pcl_pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub);
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);
};

}  // namespace extrinsic_map_base_calibrator
#endif  // EXTRINSIC_MAP_BASED_CALIBRATOR__EXTRINSIC_MAP_BASED_CALIBRATOR_HPP_
