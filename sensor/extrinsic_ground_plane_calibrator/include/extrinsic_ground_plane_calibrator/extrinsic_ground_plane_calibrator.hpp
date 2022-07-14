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

#ifndef EXTRINSIC_GROUND_PLANE_CALIBRATOR_EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_
#define EXTRINSIC_GROUND_PLANE_CALIBRATOR_EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_

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
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#ifdef ROS_DISTRO_GALACTIC
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#else
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#endif

#include <iostream>
#include <mutex>
#include <string>
#include <vector>

using PointType = pcl::PointXYZ;

class ExtrinsicGroundPlaneCalibrator : public rclcpp::Node
{
public:
  ExtrinsicGroundPlaneCalibrator(const rclcpp::NodeOptions & options);

protected:
  void requestReceivedCallback(
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
    const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response);

  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr pc);

  bool checkInitialTransforms();

  bool extractGroundPlane(
    pcl::PointCloud<PointType>::Ptr & pointcloud, Eigen::Vector4d & model,
    pcl::PointCloud<PointType>::Ptr & inliers);

  void evaluateModels(
    const Eigen::Vector4d & estimated_model, pcl::PointCloud<PointType>::Ptr inliers) const;

  void visualizeCalibration(const Eigen::Vector4d & ground_plane_model);
  void visualizePlaneModel(
    const std::string & name, Eigen::Vector4d model, visualization_msgs::msg::MarkerArray & makers);
  void visualizePlaneModel(
    const std::string & name, const Eigen::Isometry3d & lidar_base_pose,
    visualization_msgs::msg::MarkerArray & makers);

  void publishTf(const Eigen::Vector4d & ground_plane_model);

  Eigen::Vector4d poseToPlaneModel(const Eigen::Isometry3d & pose) const;
  Eigen::Isometry3d modelPlaneToPose(const Eigen::Vector4d & model) const;

  // Parameters
  std::string base_frame_;
  std::string sensor_kit_frame_;  // the parent for this calibration method must be a sensor kit
  std::string lidar_base_frame_;  // the child for this calibration method must be a the base of a
                                  // lidar (probably different from the actua lidar tf)
  double marker_size_;
  double max_inlier_distance_;
  int min_plane_points_;
  double max_cos_distance_;
  int max_iterations_;
  bool verbose_;
  bool broacast_calibration_tf_;
  bool filter_estimations_;
  double initial_angle_cov_;
  double initial_z_cov_;
  double angle_measurement_cov_;
  double angle_process_cov_;
  double z_measurement_cov_;
  double z_process_cov_;
  double angle_convergence_threshold_;
  double z_convergence_threshold_;

  // ROS Interface
  tf2_ros::StaticTransformBroadcaster tf_broascaster_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> transform_listener_;

  rclcpp::CallbackGroup::SharedPtr subs_callback_group_;
  rclcpp::CallbackGroup::SharedPtr srv_callback_group_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inliers_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;

  // Threading, sync, and result
  std::mutex mutex_;

  // ROS Data
  std_msgs::msg::Header header_;
  std::string lidar_frame_;

  // Initial tfs comparable with the one with our method
  geometry_msgs::msg::Transform initial_base_to_lidar_msg_;
  tf2::Transform initial_base_to_lidar_tf2_;
  Eigen::Isometry3d initial_base_to_lidar_eigen_;

  // Other tfs to calculate the complete chain. There are constant for our pourposes
  geometry_msgs::msg::Transform base_to_sensor_kit_msg_;
  tf2::Transform base_to_sensor_kit_tf2_;
  Eigen::Isometry3d base_to_sensor_kit_eigen_;

  geometry_msgs::msg::Transform lidar_base_to_lidar_msg_;
  tf2::Transform lidar_base_to_lidar_tf2_;
  Eigen::Isometry3d lidar_base_to_lidar_eigen_;

  geometry_msgs::msg::Pose output_calibration_msg_;

  bool got_initial_transform_;
  bool broadcast_tf_;
  bool calibration_done_;

  // Filtering
  KalmanFilter kalman_filter_;
  bool first_observation_;
};

#endif  // EXTRINSIC_GROUND_PLANE_CALIBRATOR_EXTRINSIC_GROUND_PLANE_CALIBRATOR_HPP_
