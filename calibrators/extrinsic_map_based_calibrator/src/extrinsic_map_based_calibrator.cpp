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

#include "extrinsic_map_based_calibrator/extrinsic_map_based_calibrator.hpp"

#include <limits>
#include <memory>

namespace extrinsic_map_base_calibrator
{
ExtrinsicMapBasedCalibrator::ExtrinsicMapBasedCalibrator(const rclcpp::NodeOptions & node_options)
: Node("extrinsic_map_based_calibrator", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_)
{
  using std::chrono_literals::operator""s;

  // set launch param
  parent_frame_ = this->declare_parameter("parent_frame", "");
  child_frame_ = this->declare_parameter("child_frame", "");
  is_debug_pub_ = this->declare_parameter<bool>("map_based_calibrator.debug_pub");
  is_calibration_area_map_ = this->declare_parameter<bool>("use_calibration_area_map", "");

  {
    PreprocessingConfig config{};
    config.ransac_config.max_iteration =
      this->declare_parameter<int>("preprocessing.ransac.max_iteration");
    config.ransac_config.voxel_grid_size =
      this->declare_parameter<double>("preprocessing.ransac.voxel_grid_size");
    config.ransac_config.distance_threshold =
      this->declare_parameter<double>("preprocessing.ransac.distance_threshold");
    config.clip_config.clipping_threshold =
      this->declare_parameter<double>("preprocessing.clip_wall_pointcloud.clipping_threshold");
    config.clip_config.matching_config.maximum_iteration_ =
      this->declare_parameter<int>("preprocessing.clip_wall_pointcloud.max_iteration");
    config.clip_config.matching_config.max_correspondence_distance =
      this->declare_parameter<double>(
        "preprocessing.clip_wall_pointcloud.max_correspondence_distance");
    config.clip_config.matching_config.transformation_epsilon =
      this->declare_parameter<double>("preprocessing.clip_wall_pointcloud.transformation_epsilon");
    config.clip_config.matching_config.euclidean_fitness_epsilon = this->declare_parameter<double>(
      "preprocessing.clip_wall_pointcloud.euclidean_fitness_epsilon");
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
    config.z_range_max_ = this->declare_parameter<double>("grid_search.z_range_max");
    config.z_range_min_ = this->declare_parameter<double>("grid_search.z_range_min");
    config.z_resolution_ = this->declare_parameter<double>("grid_search.z_range_resolution");
    config.roll_range_max_ = this->declare_parameter<double>("grid_search.roll_range_max");
    config.roll_range_min_ = this->declare_parameter<double>("grid_search.roll_range_min");
    config.roll_resolution_ = this->declare_parameter<double>("grid_search.roll_range_resolution");
    config.pitch_range_max_ = this->declare_parameter<double>("grid_search.pitch_range_max");
    config.pitch_range_min_ = this->declare_parameter<double>("grid_search.pitch_range_min");
    config.pitch_resolution_ =
      this->declare_parameter<double>("grid_search.pitch_range_resolution");
    config.yaw_range_max_ = this->declare_parameter<double>("grid_search.yaw_range_max");
    config.yaw_range_min_ = this->declare_parameter<double>("grid_search.yaw_range_min");
    config.yaw_resolution_ = this->declare_parameter<double>("grid_search.yaw_range_resolution");

    config.matching_config.maximum_iteration_ =
      this->declare_parameter<int>("grid_search.maximum_iteration");
    config.matching_config.max_correspondence_distance =
      this->declare_parameter<double>("grid_search.max_correspondence_distance");
    config.matching_config.transformation_epsilon =
      this->declare_parameter<double>("grid_search.transformation_epsilon");
    config.matching_config.euclidean_fitness_epsilon =
      this->declare_parameter<double>("grid_search.euclidean_fitness_epsilon");
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
      &ExtrinsicMapBasedCalibrator::targetMapWithoutWallCallback, this, std::placeholders::_1),
    subscription_option);
  source_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/source_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicMapBasedCalibrator::sourcePointcloudCallback, this, std::placeholders::_1),
    subscription_option);

  // initialize service server
  server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration", std::bind(
                               &ExtrinsicMapBasedCalibrator::requestReceivedCallback, this,
                               std::placeholders::_1, std::placeholders::_2));

  if (is_debug_pub_) {
    map_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/map_pointcloud", map_qos);
    map_without_wall_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "debug/map_without_wall_pointcloud", map_qos);
    sensor_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/sensor_pointcloud", map_qos);
    sensor_pointcloud_without_wall_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "debug/sensor_pointcloud_without_wall", map_qos);
    calibrated_pointcloud_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("debug/calibrated_pointcloud", map_qos);
  }
  // wait for other node
  rclcpp::sleep_for(10s);
}

bool ExtrinsicMapBasedCalibrator::mapBasedCalibration(const tf2::Transform & tf_initial_pose)
{
  if (!map_with_wall_pointcloud_msg_) {
    RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
    return false;
  }
  if (!map_without_wall_pointcloud_msg_ && is_calibration_area_map_) {
    RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
    return false;
  }
  if (!sensor_pointcloud_msg_) {
    RCLCPP_ERROR(this->get_logger(), "Can not received pandar left upper point cloud topic");
    return false;
  }

  if (map_with_wall_pointcloud_msg_->height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
    return false;
  }
  if (is_calibration_area_map_) {
    if (map_without_wall_pointcloud_msg_->height == 0) {
      RCLCPP_ERROR(this->get_logger(), "Can not received point cloud map topic");
      return false;
    }
  }
  if (sensor_pointcloud_msg_->height == 0) {
    RCLCPP_ERROR(this->get_logger(), "Can not received pandar left upper point cloud topic");
    return false;
  }

  PointCloudT::Ptr pcl_map(new PointCloudT);
  PointCloudT::Ptr pcl_map_without_wall(new PointCloudT);
  PointCloudT::Ptr pcl_sensor(new PointCloudT);
  if (is_calibration_area_map_) {
    if (!preprocessing(pcl_map, pcl_map_without_wall, pcl_sensor, tf_initial_pose)) {
      return false;
    }
    grid_search_matching_.executeGridSearchMatching(pcl_map_without_wall, pcl_sensor);
  } else {
    if (!preprocessing(pcl_map, pcl_sensor, tf_initial_pose)) {
      return false;
    }
    grid_search_matching_.executeGridSearchMatching(pcl_map, pcl_sensor);
  }

  calibrated_sensor_result_ = grid_search_matching_.getRematchedResult();

  PointCloudT::Ptr calibrated_pointcloud(new PointCloudT);
  pcl::transformPointCloud(
    *pcl_sensor, *calibrated_pointcloud, calibrated_sensor_result_.transformation_matrix);

  publishPointCloud(calibrated_pointcloud, calibrated_pointcloud_pub_);
  pcl::toROSMsg(*calibrated_pointcloud, calibrated_pointcloud_msg_);

  return true;
}

bool ExtrinsicMapBasedCalibrator::preprocessing(
  PointCloudT::Ptr & pcl_map, PointCloudT::Ptr & pcl_map_without_wall,
  PointCloudT::Ptr & pcl_sensor, const tf2::Transform & tf_initial_pose)
{
  PointCloudT::Ptr pcl_map_tmp(new PointCloudT);
  PointCloudT::Ptr pcl_map_without_wall_tmp(new PointCloudT);
  PointCloudT::Ptr pcl_sensor_tmp(new PointCloudT);
  if (
    !convertFromROSMsg(pcl_map_tmp, map_with_wall_pointcloud_msg_) ||
    !convertFromROSMsg(pcl_map_without_wall_tmp, map_without_wall_pointcloud_msg_) ||
    !convertFromROSMsg(pcl_sensor_tmp, sensor_pointcloud_msg_)) {
    RCLCPP_ERROR(this->get_logger(), "Fault convert ros message to pcl");
    return false;
  }
  // transform each frame to parent_frame
  PointCloudT::Ptr transformed_sensor(new PointCloudT);
  pcl_ros::transformPointCloud(parent_frame_, *pcl_map_tmp, *pcl_map, tf_buffer_);
  pcl_ros::transformPointCloud(
    parent_frame_, *pcl_map_without_wall_tmp, *pcl_map_without_wall, tf_buffer_);
  // std::cout << "map without wall points " << pcl_map_without_wall->points.size() << std::endl;

  pcl_ros::transformPointCloud(*pcl_sensor_tmp, *transformed_sensor, tf_initial_pose);

  publishPointCloud(pcl_map, map_pointcloud_pub_);
  publishPointCloud(pcl_map_without_wall, map_without_wall_pointcloud_pub_);
  publishPointCloud(transformed_sensor, sensor_pointcloud_pub_);

  pcl_sensor = preprocessing_.preprocessing(pcl_map, pcl_map_without_wall, transformed_sensor);
  publishPointCloud(pcl_sensor, sensor_pointcloud_without_wall_pub_);
  return true;
}

bool ExtrinsicMapBasedCalibrator::preprocessing(
  PointCloudT::Ptr & pcl_map, PointCloudT::Ptr & pcl_sensor, const tf2::Transform & tf_initial_pose)
{
  PointCloudT::Ptr pcl_map_tmp(new PointCloudT);
  PointCloudT::Ptr pcl_sensor_tmp(new PointCloudT);
  if (
    !convertFromROSMsg(pcl_map_tmp, map_with_wall_pointcloud_msg_) ||
    !convertFromROSMsg(pcl_sensor_tmp, sensor_pointcloud_msg_)) {
    RCLCPP_ERROR(this->get_logger(), "Fault convert ros message to pcl");
    return false;
  }
  // transform each frame to parent_frame
  PointCloudT::Ptr transformed_sensor(new PointCloudT);
  pcl_ros::transformPointCloud(parent_frame_, *pcl_map_tmp, *pcl_map, tf_buffer_);
  pcl_ros::transformPointCloud(*pcl_sensor_tmp, *transformed_sensor, tf_initial_pose);

  publishPointCloud(pcl_map, map_pointcloud_pub_);
  publishPointCloud(transformed_sensor, sensor_pointcloud_pub_);

  pcl_sensor = preprocessing_.preprocessing(pcl_map, transformed_sensor);
  publishPointCloud(pcl_sensor, sensor_pointcloud_without_wall_pub_);
  return true;
}

bool ExtrinsicMapBasedCalibrator::convertFromROSMsg(
  PointCloudT::Ptr & pcl_pointcloud, const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  {
    std::lock_guard<std::mutex> message_lock(mutex_);
    if (!msg) {
      return false;
    }
    pcl::fromROSMsg(*msg, *pcl_pointcloud);
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
  const PointCloudT::Ptr & pcl_map, const PointCloudT::Ptr & pcl_map_without_wall,
  const PointCloudT::Ptr & pcl_sensor, const PointCloudT::Ptr & pcl_sensor_without_wall,
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
  if (!is_debug_pub_) {
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
  using std::chrono_literals::operator""s;

  // Wait for subscription topic
  while (rclcpp::ok() && (!sensor_pointcloud_msg_ || !map_with_wall_pointcloud_msg_ ||
                          (!map_without_wall_pointcloud_msg_ && is_calibration_area_map_))) {
    if (!map_with_wall_pointcloud_msg_) {
      RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Can not received point cloud map topic");
    } else if (!map_without_wall_pointcloud_msg_ && is_calibration_area_map_) {
      RCLCPP_WARN_SKIPFIRST(
        this->get_logger(), "Can not received point cloud map without wall topic");
    } else if (!sensor_pointcloud_msg_) {
      RCLCPP_WARN_SKIPFIRST(this->get_logger(), "Can not received sensor point cloud topic");
    }
    rclcpp::sleep_for(1s);
  }

  // set initial pose
  tf2::Transform tf_initial_pose;
  tf2::fromMsg(request->initial_pose, tf_initial_pose);

  // execute gicp matching
  RCLCPP_DEBUG_STREAM(this->get_logger(), "--- Execute map based calibration ---");
  bool is_matching = mapBasedCalibration(tf_initial_pose);
  response->success = is_matching;

  // set result to response
  // printTransform(tf_initial_pose);
  response->debug_pointcloud.header.frame_id = parent_frame_;
  response->debug_pointcloud = calibrated_pointcloud_msg_;
  matchingResult prematch = preprocessing_.getPrematchedResult();
  tf2::Vector3 trans_pre(
    prematch.transformation_matrix(0, 3), prematch.transformation_matrix(1, 3),
    prematch.transformation_matrix(2, 3));
  Eigen::Matrix3d R_pre = prematch.transformation_matrix.block(0, 0, 3, 3);
  Eigen::Quaterniond q_pre(R_pre);

  tf2::Quaternion Qtf_pre(q_pre.x(), q_pre.y(), q_pre.z(), q_pre.w());
  tf2::Transform tf_prematch(Qtf_pre, trans_pre);

  tf2::Vector3 trans(
    calibrated_sensor_result_.transformation_matrix(0, 3),
    calibrated_sensor_result_.transformation_matrix(1, 3),
    calibrated_sensor_result_.transformation_matrix(2, 3));
  Eigen::Matrix3d R = calibrated_sensor_result_.transformation_matrix.block(0, 0, 3, 3);
  Eigen::Quaterniond q(R);

  tf2::Quaternion Qtf(q.x(), q.y(), q.z(), q.w());
  tf2::Transform result_tf(Qtf, trans);
  // printTransform(result_tf);
  result_tf = result_tf * tf_prematch * tf_initial_pose;
  // printTransform(result_tf);
  tf2::toMsg(result_tf, response->result_pose);

  response->score = calibrated_sensor_result_.score;
}

void ExtrinsicMapBasedCalibrator::printTransform(const tf2::Transform & tf)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "trans");
  RCLCPP_INFO_STREAM(
    this->get_logger(), tf.getOrigin().x()
                          << ", " << tf.getOrigin().y() << ", " << tf.getOrigin().z());

  tf2::Matrix3x3 tf2_matrix(tf.getRotation());
  double roll;
  double pitch;
  double yaw;
  tf2_matrix.getRPY(roll, pitch, yaw);
  RCLCPP_INFO_STREAM(this->get_logger(), "rotate");
  RCLCPP_INFO_STREAM(this->get_logger(), roll << ", " << pitch << ", " << yaw);
}

}  // namespace extrinsic_map_base_calibrator

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node =
    std::make_shared<extrinsic_map_base_calibrator::ExtrinsicMapBasedCalibrator>(node_options);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
