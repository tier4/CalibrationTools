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

#include "extrinsic_map_based_calibrator/extrinsic_map_based_calibrator.hpp"
#include <memory>
#include <limits>

using namespace std::chrono_literals;

namespace extrinsic_map_base_calibrator
{
ExtrinsicMapBasedCalibrator::ExtrinsicMapBasedCalibrator(const rclcpp::NodeOptions & node_options)
: Node("extrinsic_map_based_calibrator", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  // set launch param
  parent_frame_ = this->declare_parameter("parent_frame", "");
  child_frame_ = this->declare_parameter("child_frame", "");
  is_debug_pub_ = this->declare_parameter<bool>("map_based_calibrator.debug_pub");

  {
    PreprocessingConfig config;
    config.ransac_config.max_iteration = this->declare_parameter<int>("preprocessing.ransac.max_iteration");
    config.ransac_config.voxel_grid_size = this->declare_parameter<double>("preprocessing.ransac.voxel_grid_size");
    config.ransac_config.distance_threshold = this->declare_parameter<double>("preprocessing.ransac.distance_threshold");
    config.clip_config.clipping_threshold
      = this->declare_parameter<double>("preprocessing.clip_wall_pointcloud.clipping_threshold");
    config.clip_config.matching_config.maximum_iteration_
      = this->declare_parameter<int>("preprocessing.clip_wall_pointcloud.max_iteration");
    config.clip_config.matching_config.max_correspondence_distance
      = this->declare_parameter<double>("preprocessing.clip_wall_pointcloud.max_correspondence_distance");
    config.clip_config.matching_config.transformation_epsilon
      = this->declare_parameter<double>("preprocessing.clip_wall_pointcloud.transformation_epsilon");
    config.clip_config.matching_config.euclidean_fitness_epsilon
      = this->declare_parameter<double>("preprocessing.clip_wall_pointcloud.euclidean_fitness_epsilon");
    preprocessing_.setConfig(config);
  }
  {
    GridSearchConfig config;
    config.x_range_max_ = this->declare_parameter<double>("grid_search.x_range_max");
    config.x_range_min_ = this->declare_parameter<double>("grid_search.x_range_min");
    config.x_resolution_ = this->declare_parameter<double>("grid_search.x_range_resolution");
    config.y_range_max_ = this->declare_parameter<double>("grid_search.y_range_max");
    config.y_range_min_ = this->declare_parameter<double>("grid_search.y_range_min");
    config.y_resolution_ = this->declare_parameter<double>("grid_search.y_range_resolution");
    config.yaw_range_max_ = this->declare_parameter<double>("grid_search.yaw_range_max");
    config.yaw_range_min_ = this->declare_parameter<double>("grid_search.yaw_range_min");
    config.yaw_resolution_ = this->declare_parameter<double>("grid_search.yaw_range_resolution");

    config.matching_config.maximum_iteration_
      = this->declare_parameter<int>("grid_search.maximum_iteration");
    config.matching_config.max_correspondence_distance
      = this->declare_parameter<double>("grid_search.max_correspondence_distance");
    config.matching_config.transformation_epsilon
      = this->declare_parameter<double>("grid_search.transformation_epsilon");
    config.matching_config.euclidean_fitness_epsilon
      = this->declare_parameter<double>("grid_search.euclidean_fitness_epsilon");
    grid_search_matching_.setParameter(config);
  }

  callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  auto subscription_option = rclcpp::SubscriptionOptions();
  subscription_option.callback_group = callback_group_;

  // QoS setup
  static constexpr std::size_t queue_size = 1;
  rclcpp::QoS map_qos(queue_size);
  map_qos.transient_local();  // option for latching

  // initialize publisher and subscriber
  map_with_wall_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud_map_with_wall", map_qos,
    std::bind(&ExtrinsicMapBasedCalibrator::targetMapWithWallCallback, this, std::placeholders::_1),
    subscription_option);
  map_without_wall_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/pointcloud_map_without_wall", map_qos,
    std::bind(
      &ExtrinsicMapBasedCalibrator::targetMapWithoutWallCallback, this,
      std::placeholders::_1),
    subscription_option);
  source_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/source_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(
      &ExtrinsicMapBasedCalibrator::sourcePointcloudCallback, this,
      std::placeholders::_1), subscription_option);

  // initialize service server
  server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration", std::bind(
      &ExtrinsicMapBasedCalibrator::requestReceivedCallback, this,
      std::placeholders::_1, std::placeholders::_2));

  if (is_debug_pub_) {
    map_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/map_pointcloud", map_qos);
    map_without_wall_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/map_without_wall_pointcloud", map_qos);
    sensor_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/sensor_pointcloud", map_qos);
    sensor_pointcloud_without_wall_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/sensor_pointcloud_without_wall", map_qos);
    calibrated_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/calibrated_pointcloud", map_qos);
  }
}

bool ExtrinsicMapBasedCalibrator::mapBasedCalibration(const tf2::Transform & tf_initial_pose)
{
  if (!map_with_wall_pointcloud_msg_) {
    RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
    return false;
  } else if (!map_without_wall_pointcloud_msg_) {
    RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
    return false;
  } else if (!sensor_pointcloud_msg_) {
    RCLCPP_ERROR(this->get_logger(), "Can not received pandar left upper point cloud topic");
    return false;
  }
  if (map_with_wall_pointcloud_msg_->height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
    return false;
  } else if (map_without_wall_pointcloud_msg_->height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
    return false;
  } else if (sensor_pointcloud_msg_->height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Can not received pandar left upper point cloud topic");
    return false;
  }

  PointCloudT::Ptr pcl_map(new PointCloudT);
  PointCloudT::Ptr pcl_map_without_wall(new PointCloudT);
  PointCloudT::Ptr pcl_sensor(new PointCloudT);
  if (!preprocessing(pcl_map, pcl_map_without_wall, pcl_sensor, tf_initial_pose)) {
    return false;
  }

  grid_search_matching_.executeGridSearchMatching(pcl_map_without_wall, pcl_sensor);

  matchingResult final_result = grid_search_matching_.getRematchedResult();

  PointCloudT::Ptr calibrated_pointcloud(new PointCloudT);
  pcl::transformPointCloud(*pcl_sensor, *calibrated_pointcloud, final_result.transformation_matrix);

  publishPointCloud(calibrated_pointcloud, calibrated_pointcloud_pub_);
  pcl::toROSMsg(*calibrated_pointcloud, calibrated_pointcloud_msg_);

  return true;
}

bool ExtrinsicMapBasedCalibrator::preprocessing(
  PointCloudT::Ptr & pcl_map,
  PointCloudT::Ptr & pcl_map_without_wall,
  PointCloudT::Ptr & pcl_sensor, const tf2::Transform & tf_initial_pose)
{
  PointCloudT::Ptr pcl_map_tmp(new PointCloudT);
  PointCloudT::Ptr pcl_map_without_wall_tmp(new PointCloudT);
  PointCloudT::Ptr pcl_sensor_tmp(new PointCloudT);
  if (!convertFromROSMsg(pcl_map_tmp, pcl_map_without_wall_tmp, pcl_sensor_tmp)) {
    return false;
  }
  // transform each frame to parent_frame
  PointCloudT::Ptr transformed_sensor(new PointCloudT);
  pcl_ros::transformPointCloud(parent_frame_, *pcl_map_tmp, *pcl_map, tf_buffer_);
  pcl_ros::transformPointCloud(
    parent_frame_, *pcl_map_without_wall_tmp, *pcl_map_without_wall,
    tf_buffer_);
  // std::cout << "map without wall points " << pcl_map_without_wall->points.size() << std::endl;

  pcl_ros::transformPointCloud(*pcl_sensor_tmp, *transformed_sensor, tf_initial_pose);

  publishPointCloud(pcl_map, map_pointcloud_pub_);
  publishPointCloud(pcl_map_without_wall, map_without_wall_pointcloud_pub_);
  publishPointCloud(transformed_sensor, sensor_pointcloud_pub_);

  pcl_sensor = preprocessing_.preprocessing(pcl_map, pcl_map_without_wall, transformed_sensor);
  publishPointCloud(pcl_sensor, sensor_pointcloud_without_wall_pub_);
  return true;
}

bool ExtrinsicMapBasedCalibrator::convertFromROSMsg(
  PointCloudT::Ptr & pcl_map,
  PointCloudT::Ptr & pcl_map_without_wall,
  PointCloudT::Ptr & pcl_sensor)
{
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    if (!map_with_wall_pointcloud_msg_) {
      RCLCPP_ERROR(this->get_logger(), "cant get point cloud map");
      return false;
    }
    pcl::fromROSMsg(*map_with_wall_pointcloud_msg_, *pcl_map);
  }
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    if (!map_without_wall_pointcloud_msg_) {
      RCLCPP_ERROR(this->get_logger(), "cant get point cloud map without wall");
      return false;
    }
    pcl::fromROSMsg(*map_without_wall_pointcloud_msg_, *pcl_map_without_wall);
  }
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    if (!sensor_pointcloud_msg_) {
      RCLCPP_ERROR(this->get_logger(), "cant get sensor cloud");
      return false;
    }
    pcl::fromROSMsg(*sensor_pointcloud_msg_, *pcl_sensor);
  }
  return true;
}

void ExtrinsicMapBasedCalibrator::targetMapWithWallCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> message_lock(mutex_);
  map_with_wall_pointcloud_msg_ = msg;
}

void ExtrinsicMapBasedCalibrator::targetMapWithoutWallCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> message_lock(mutex_);
  map_without_wall_pointcloud_msg_ = msg;
}

void ExtrinsicMapBasedCalibrator::sourcePointcloudCallback(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  std::lock_guard<std::mutex> message_lock(mutex_);
  sensor_pointcloud_msg_ = msg;
}

void ExtrinsicMapBasedCalibrator::publishPointCloud(
  const PointCloudT::Ptr & pcl_map,
  const PointCloudT::Ptr & pcl_map_without_wall,
  const PointCloudT::Ptr & pcl_sensor,
  const PointCloudT::Ptr & pcl_sensor_without_wall,
  const PointCloudT::Ptr & pcl_result)
{
  publishPointCloud(pcl_map, map_pointcloud_pub_);
  publishPointCloud(pcl_map_without_wall, map_without_wall_pointcloud_pub_);
  publishPointCloud(pcl_sensor, sensor_pointcloud_pub_);
  publishPointCloud(pcl_sensor_without_wall, sensor_pointcloud_without_wall_pub_);
  publishPointCloud(pcl_result, calibrated_pointcloud_pub_);
}

void ExtrinsicMapBasedCalibrator::publishPointCloud(
  const PointCloudT::Ptr & pcl_pointcloud,
  const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr & pub)
{
  if( !is_debug_pub_ ){
    return;
  }
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*pcl_pointcloud, pointcloud_msg);
  pointcloud_msg.header.frame_id = parent_frame_;
  pointcloud_msg.header.stamp = this->now();
  pub->publish(pointcloud_msg);
}

void ExtrinsicMapBasedCalibrator::requestReceivedCallback(
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request> request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  // Wait for subscription topic
  while (rclcpp::ok() &&
    (!sensor_pointcloud_msg_ || !map_with_wall_pointcloud_msg_ ||
    !map_without_wall_pointcloud_msg_ ))
  {
    if (!map_with_wall_pointcloud_msg_) {
      RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Can not received point cloud map topic");
    } else if (!map_without_wall_pointcloud_msg_) {
      RCLCPP_WARN_SKIPFIRST(
        this->get_logger(), "Can not received point cloud map without wall topic");
    } else if (!sensor_pointcloud_msg_) {
      RCLCPP_WARN_SKIPFIRST(
        this->get_logger(), "map size %ld", map_with_wall_pointcloud_msg_->data.size());
      RCLCPP_WARN_SKIPFIRST(
        this->get_logger(), "Can not received sensor point cloud topic");
    }
    rclcpp::sleep_for(1s);
  }

  // set initial pose
  tf2::Transform tf_initial_pose;
  tf2::fromMsg(request->initial_pose, tf_initial_pose);

  // execute gicp matching
  RCLCPP_DEBUG_STREAM(this->get_logger(), "--- Execute gicp Matching ---");
  bool is_matching = mapBasedCalibration(tf_initial_pose);
  if ( is_matching) {
    response->success = true;
  } else {
    response->success = false;
  }

  // set result to response
  response->debug_pointcloud.header.frame_id = parent_frame_;
  response->debug_pointcloud = calibrated_pointcloud_msg_;
  response->result_pose.position.x = calibrated_sensor_result_.transformation_matrix(0, 3);
  response->result_pose.position.y = calibrated_sensor_result_.transformation_matrix(1, 3);
  response->result_pose.position.z = calibrated_sensor_result_.transformation_matrix(2, 3);
  Eigen::Matrix3d R = calibrated_sensor_result_.transformation_matrix.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);
  response->result_pose.orientation.x = q.x();
  response->result_pose.orientation.y = q.y();
  response->result_pose.orientation.z = q.z();
  response->result_pose.orientation.w = q.w();

  response->score = calibrated_sensor_result_.score;
}

}  // namespace extrinsic_map_base_calibrator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<extrinsic_map_base_calibrator::ExtrinsicMapBasedCalibrator>(
    node_options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
