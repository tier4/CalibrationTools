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

#include <extrinsic_reflector_based_calibrator/extrinsic_reflector_based_calibrator.hpp>
#include <tier4_autoware_utils/tier4_autoware_utils.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>

#ifdef ROS_DISTRO_GALACTIC
#include <tf2_eigen/tf2_eigen.h>
#else
#include <tf2_eigen/tf2_eigen.hpp>
#endif

#include <iostream>
#include <limits>

#define UNUSED(x) (void)x;

ExtrinsicReflectorBasedCalibrator::ExtrinsicReflectorBasedCalibrator(
  const rclcpp::NodeOptions & options)
: Node("extrinsic_ground_plane_calibrator_node", options),
  tf_broadcaster_(this),
  got_initial_transform_(false),
  calibration_done_(false),
  first_observation_(true),
  extract_lidar_background_model_(false),
  extract_radar_background_model_(false)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  sensor_kit_frame_ = this->declare_parameter<std::string>("parent_frame");
  lidar_base_frame_ = this->declare_parameter<std::string>("child_frame");

  lidar_background_model_.leaf_size_ =
    this->declare_parameter<double>("lidar_background_model_leaf_size", 0.1);
  radar_background_model_.leaf_size_ =
    this->declare_parameter<double>("radar_background_model_leaf_size", 0.1);
  background_model_margin_ = this->declare_parameter<double>("background_model_margin", 0.01);
  background_model_timeout_ = this->declare_parameter<double>("background_model_timeout", 5.0);
  min_foreground_distance_ = this->declare_parameter<double>("min_foreground_distance", 0.2);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);

  lidar_background_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_background_pointcloud", 10);
  lidar_foreground_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_foreground_pointcloud", 10);

  radar_background_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_background_pointcloud", 10);
  radar_foreground_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_foreground_pointcloud", 10);

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicReflectorBasedCalibrator::lidarCallback, this, std::placeholders::_1));

  radar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicReflectorBasedCalibrator::radarCallback, this, std::placeholders::_1));

  // The service server runs in a dedicated thread
  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  calibration_request_server_ =
    this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
      "extrinsic_calibration",
      std::bind(
        &ExtrinsicReflectorBasedCalibrator::requestReceivedCallback, this, std::placeholders::_1,
        std::placeholders::_2),
      rmw_qos_profile_services_default, srv_callback_group_);

  background_model_server_ = this->create_service<std_srvs::srv::Empty>(
    "background_model_service",
    std::bind(
      &ExtrinsicReflectorBasedCalibrator::backgroundModelRequestCallback, this,
      std::placeholders::_1, std::placeholders::_2),
    rmw_qos_profile_services_default, srv_callback_group_);

  // Initialize the filter
  lidar_filter_.setA(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  lidar_filter_.setB(Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0));
  lidar_filter_.setC(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  lidar_filter_.setQ(Eigen::DiagonalMatrix<double, 3>(
    lidar_measurement_cov_, lidar_measurement_cov_, lidar_measurement_cov_));
  lidar_filter_.setR(
    Eigen::DiagonalMatrix<double, 3>(lidar_process_cov_, lidar_process_cov_, lidar_process_cov_));

  radar_filter_.setA(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  radar_filter_.setB(Eigen::DiagonalMatrix<double, 3>(0.0, 0.0, 0.0));
  radar_filter_.setC(Eigen::DiagonalMatrix<double, 3>(1.0, 1.0, 1.0));
  radar_filter_.setQ(Eigen::DiagonalMatrix<double, 3>(
    radar_measurement_cov_, radar_measurement_cov_, radar_measurement_cov_));
  radar_filter_.setR(
    Eigen::DiagonalMatrix<double, 3>(radar_process_cov_, radar_process_cov_, radar_process_cov_));

  lidar_background_model_.tree_ = std::make_shared<pcl::octree::OctreePointCloudSearch<PointType>>(
    lidar_background_model_.leaf_size_);
  radar_background_model_.tree_ = std::make_shared<pcl::octree::OctreePointCloudSearch<PointType>>(
    radar_background_model_.leaf_size_);
}

void ExtrinsicReflectorBasedCalibrator::requestReceivedCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  // This tool uses several tfs, so for consistency we take the initial calibration using lookups
  UNUSED(request);
  using std::chrono_literals::operator""s;

  // Loop until the calibration finishes
  while (rclcpp::ok()) {
    rclcpp::sleep_for(10s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (calibration_done_) {
      break;
    }

    RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Waiting for the calibration to end");
  }

  response->success = true;
  response->result_pose = output_calibration_msg_;
}

void ExtrinsicReflectorBasedCalibrator::backgroundModelRequestCallback(
  const std::shared_ptr<std_srvs::srv::Empty::Request> request,
  const std::shared_ptr<std_srvs::srv::Empty::Response> response)
{
  UNUSED(request);
  UNUSED(response);
  using std::chrono_literals::operator""s;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    extract_lidar_background_model_ = true;
    extract_radar_background_model_ = true;
  }

  while (rclcpp::ok()) {
    rclcpp::sleep_for(5s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (lidar_background_model_.valid_ && radar_background_model_.valid_) {
      break;
    }

    RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Waiting for the background model to be computed");
  }

  RCLCPP_INFO(this->get_logger(), "Background model estimated");
}

void ExtrinsicReflectorBasedCalibrator::lidarCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  lidar_frame_ = msg->header.frame_id;
  lidar_header_ = msg->header;
  bool extract_background_model;
  bool valid_background_model;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    extract_background_model = extract_lidar_background_model_;
    valid_background_model = lidar_background_model_.valid_;
  }

  pcl::PointCloud<PointType>::Ptr lidar_pointcloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *lidar_pointcloud);

  if (extract_background_model && !valid_background_model) {
    extractBackgroundModel(
      lidar_pointcloud, msg->header, latest_updated_lidar_frame_, lidar_background_model_);
    return;
  }

  if (!valid_background_model) {
    return;
  }

  pcl::PointCloud<PointType>::Ptr foreground_pointcloud =
    extractForegroundPoints(lidar_pointcloud, lidar_background_model_);

  sensor_msgs::msg::PointCloud2 background_msg;
  pcl::toROSMsg(*lidar_background_model_.pointcloud_, background_msg);
  background_msg.header = lidar_header_;
  lidar_background_pub_->publish(background_msg);

  sensor_msgs::msg::PointCloud2 foreground_msg;
  pcl::toROSMsg(*foreground_pointcloud, foreground_msg);
  foreground_msg.header = lidar_header_;
  lidar_foreground_pub_->publish(foreground_msg);
}

void ExtrinsicReflectorBasedCalibrator::radarCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  radar_frame_ = msg->header.frame_id;
  radar_header_ = msg->header;

  pcl::PointCloud<PointType>::Ptr radar_pointcloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *radar_pointcloud);

  if (extract_radar_background_model_) {
    extractBackgroundModel(
      radar_pointcloud, msg->header, latest_updated_radar_frame_, radar_background_model_);
    return;
  }

  if (!radar_background_model_.valid_) {
    return;
  }
}

void ExtrinsicReflectorBasedCalibrator::extractBackgroundModel(
  const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
  const std_msgs::msg::Header & current_header, std_msgs::msg::Header & last_updated_header,
  BackgroundModel & background_model)
{
  // Initialize background model in the first iteration
  if (background_model.set_.size() == 0) {
    PointType min_point_pcl, max_point_pcl;
    pcl::getMinMax3D(*sensor_pointcloud, min_point_pcl, max_point_pcl);
    background_model.min_point_ =
      Eigen::Vector4f(min_point_pcl.getArray4fMap()) -
      Eigen::Vector4f(
        -background_model_margin_, -background_model_margin_, -background_model_margin_, 0.0);
    background_model.max_point_ =
      Eigen::Vector4f(max_point_pcl.getArray4fMap()) -
      Eigen::Vector4f(
        background_model_margin_, background_model_margin_, background_model_margin_, 0.0);
    last_updated_header = current_header;
  }

  index_t x_cells = (background_model.max_point_.x() - background_model.min_point_.x()) /
                    background_model.leaf_size_;
  index_t y_cells = (background_model.max_point_.y() - background_model.min_point_.y()) /
                    background_model.leaf_size_;
  index_t z_cells = (background_model.max_point_.z() - background_model.min_point_.z()) /
                    background_model.leaf_size_;
  background_model.pointcloud_->points.reserve(x_cells * y_cells * z_cells);
  bool background_model_updated = false;

  for (const auto & p : sensor_pointcloud->points) {
    index_t x_index =
      static_cast<index_t>((p.x - background_model.min_point_.x()) / background_model.leaf_size_);
    index_t y_index =
      static_cast<index_t>((p.y - background_model.min_point_.y()) / background_model.leaf_size_);
    index_t z_index =
      static_cast<index_t>((p.z - background_model.min_point_.z()) / background_model.leaf_size_);
    index_t index = z_index * y_cells * x_cells + y_index * x_cells + x_index;
    const auto & it = background_model.set_.emplace(index);

    if (it.second) {
      background_model_updated = true;
      PointType p_center;
      p_center.x = background_model.min_point_.x() + background_model.leaf_size_ * (x_index + 0.5f);
      p_center.y = background_model.min_point_.y() + background_model.leaf_size_ * (y_index + 0.5f);
      p_center.z = background_model.min_point_.z() + background_model.leaf_size_ * (z_index + 0.5f);
      background_model.pointcloud_->push_back(p_center);
    }
  }

  if (background_model_updated) {
    last_updated_header = current_header;
    return;
  }

  if (
    (rclcpp::Time(current_header.stamp) - rclcpp::Time(last_updated_header.stamp)).seconds() <
    background_model_timeout_) {
    return;
  }

  background_model.tree_->setInputCloud(background_model.pointcloud_);
  background_model.tree_->addPointsFromInputCloud();
  background_model.pointcloud_->points.shrink_to_fit();

  {
    std::unique_lock<std::mutex> lock(mutex_);
    background_model.valid_ = true;
  }
}

pcl::PointCloud<ExtrinsicReflectorBasedCalibrator::PointType>::Ptr
ExtrinsicReflectorBasedCalibrator::extractForegroundPoints(
  const pcl::PointCloud<PointType>::Ptr & sensor_pointcloud,
  const BackgroundModel & background_model)
{
  // Crop box
  pcl::PointCloud<PointType>::Ptr cropped_pointcloud(new pcl::PointCloud<PointType>);
  pcl::CropBox<PointType> crop_filter;
  crop_filter.setMin(background_model.min_point_);
  crop_filter.setMax(background_model.max_point_);
  crop_filter.setInputCloud(sensor_pointcloud);
  crop_filter.filter(*cropped_pointcloud);

  // Fast hash
  pcl::PointCloud<PointType>::Ptr filtered_pointcloud(new pcl::PointCloud<PointType>);
  filtered_pointcloud->reserve(cropped_pointcloud->size());

  index_t x_cells = (background_model.max_point_.x() - background_model.min_point_.x()) /
                    background_model.leaf_size_;
  index_t y_cells = (background_model.max_point_.y() - background_model.min_point_.y()) /
                    background_model.leaf_size_;

  for (const auto & p : cropped_pointcloud->points) {
    index_t x_index =
      static_cast<index_t>((p.x - background_model.min_point_.x()) / background_model.leaf_size_);
    index_t y_index =
      static_cast<index_t>((p.y - background_model.min_point_.y()) / background_model.leaf_size_);
    index_t z_index =
      static_cast<index_t>((p.z - background_model.min_point_.z()) / background_model.leaf_size_);
    index_t index = z_index * y_cells * x_cells + y_index * x_cells + x_index;

    if (background_model.set_.count(index) > 0) {
      filtered_pointcloud->emplace_back(p);
    }
  }

  // K-search
  pcl::PointCloud<PointType>::Ptr foreground_pointcloud(new pcl::PointCloud<PointType>);
  foreground_pointcloud->reserve(filtered_pointcloud->size());
  float min_foreground_square_distance = min_foreground_distance_ * min_foreground_distance_;

  for (const auto & p : filtered_pointcloud->points) {
    std::vector<int> indexes;
    std::vector<float> square_distances;

    if (background_model.tree_->nearestKSearch(p, 1, indexes, square_distances) > 0) {
      if (square_distances.size() == 0 || square_distances[0] >= min_foreground_square_distance) {
        foreground_pointcloud->emplace_back(p);
      }
    }
  }

  return foreground_pointcloud;
}
