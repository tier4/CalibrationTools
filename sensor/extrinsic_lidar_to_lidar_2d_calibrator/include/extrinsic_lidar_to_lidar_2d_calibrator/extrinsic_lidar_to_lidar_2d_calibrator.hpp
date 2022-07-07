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

#ifndef EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_HPP_
#define EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_HPP_

#define PCL_NO_PRECOMPILE  // We require this macro to use the PCL templates with velodyne PCs
#include <Eigen/Dense>
#include <kalman_filter/kalman_filter.hpp>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tier4_calibration_msgs/srv/extrinsic_calibrator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/filters/passthrough.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <velodyne_pointcloud/point_types.h>

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

// using PointType = velodyne_pointcloud::PointXYZIR;
using PointType = pcl::PointXYZ;
// using PointCloudType = pcl::PointCloud<pcl::PointXYZ>;

class LidarToLidar2DCalibrator : public rclcpp::Node
{
public:
  LidarToLidar2DCalibrator(const rclcpp::NodeOptions & options);

protected:
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  void sourcePointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);
  void targetPointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  void calibrationimerCallback();

  bool checkInitialTransforms();

  double getAlignmentError(
    pcl::PointCloud<PointType>::Ptr source_pointcloud,
    pcl::PointCloud<PointType>::Ptr target_pointcloud);

  // Parameters
  std::string base_frame_;
  std::string sensor_kit_frame_;  // the parent for this calibration method must be a sensor kit
  std::string lidar_base_frame_;  // the child for this calibration method must be a the base of a
                                  // lidar (probably different from the actua lidar tf)
  bool broacast_calibration_tf_;
  double min_z_;
  double max_z_;

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broascaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::TimerBase::SharedPtr calib_timer_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr source_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr target_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr source_pointcloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr target_pointcloud_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;

  // Threading, sync, and result
  std::mutex mutex_;
  geometry_msgs::msg::Pose initial_calibration_;

  // ROS Data
  sensor_msgs::msg::PointCloud2::SharedPtr source_pointcloud_msg_;
  sensor_msgs::msg::PointCloud2::SharedPtr target_pointcloud_msg_;
  std_msgs::msg::Header source_pointcloud_header_;
  std_msgs::msg::Header target_pointcloud_header_;
  std::string source_pointcloud_frame_;
  std::string target_pointcloud_frame_;

  // Initial tfs comparable with the one with our method
  geometry_msgs::msg::Transform initial_base_to_source_msg_;
  geometry_msgs::msg::Transform initial_base_to_target_msg_;
  tf2::Transform initial_base_to_source_tf2_;
  tf2::Transform initial_base_to_target_tf2_;
  Eigen::Affine3d initial_base_to_source_eigen_;
  Eigen::Affine3d initial_base_to_target_eigen_;

  // Other tfs to calculate the complete chain. There are constant for our pourposes
  geometry_msgs::msg::Transform base_to_sensor_kit_msg_;
  tf2::Transform base_to_sensor_kit_tf2_;
  Eigen::Affine3d base_to_sensor_kit_eigen_;

  geometry_msgs::msg::Pose output_calibration_msg_;

  bool got_initial_transform_;
  bool received_request_;
  bool broadcast_tf_;
  bool calibration_done_;
};

#endif  // EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_EXTRINSIC_LIDAR_TO_LIDAR_2D_CALIBRATOR_HPP_
