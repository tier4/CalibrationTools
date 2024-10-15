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

#include <autoware/universe_utils/geometry/geometry.hpp>
#include <ground_plane_calibrator/ground_plane_calibrator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2/utils.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace ground_plane_calibrator
{

ExtrinsicGroundPlaneCalibrator::ExtrinsicGroundPlaneCalibrator(const rclcpp::NodeOptions & options)
: Node("ground_plane_calibrator_node", options), tf_broadcaster_(this)
{
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
  lidar_frame_ = this->declare_parameter<std::string>("lidar_frame");

  marker_size_ = this->declare_parameter<double>("marker_size", 20.0);

  auto & parameters = ground_plane_extractor_parameters_;
  parameters.use_crop_box_filter_ = this->declare_parameter<bool>("use_crop_box_filter", true);
  parameters.crop_box_min_x_ = this->declare_parameter<double>("crop_box_min_x", -50.0);
  parameters.crop_box_min_y_ = this->declare_parameter<double>("crop_box_min_y", -50.0);
  parameters.crop_box_min_z_ = this->declare_parameter<double>("crop_box_min_z", -50.0);
  parameters.crop_box_max_x_ = this->declare_parameter<double>("crop_box_max_x", 50.0);
  parameters.crop_box_max_y_ = this->declare_parameter<double>("crop_box_max_y", 50.0);
  parameters.crop_box_max_z_ = this->declare_parameter<double>("crop_box_max_z", 50.0);

  parameters.remove_outliers_ = this->declare_parameter<bool>("remove_outliers", true);
  parameters.remove_outlier_tolerance_ =
    this->declare_parameter<double>("remove_outlier_tolerance", 0.1);
  parameters.use_pca_rough_normal_ = this->declare_parameter<bool>("use_pca_rough_normal", true);
  parameters.max_inlier_distance_ = this->declare_parameter<double>("max_inlier_distance", 0.01);
  parameters.min_plane_points_ = this->declare_parameter<int>("min_plane_points", 500);
  parameters.min_plane_points_percentage_ =
    this->declare_parameter<double>("min_plane_points_percentage", 10.0);
  parameters.max_cos_distance_ = this->declare_parameter<double>("max_cos_distance", 0.2);
  parameters.max_iterations_ = this->declare_parameter<int>("max_iterations", 500);
  verbose_ = this->declare_parameter<bool>("verbose", false);
  parameters.verbose_ = verbose_;
  overwrite_xy_yaw_ = this->declare_parameter<bool>("overwrite_xy_yaw", false);
  filter_estimations_ = this->declare_parameter<bool>("filter_estimations", true);
  int ring_buffer_size = this->declare_parameter<int>("ring_buffer_size", 100);

  inlier_observations_.setMaxSize(ring_buffer_size);

  initial_angle_cov_ = this->declare_parameter<double>("initial_angle_cov", 5.0);
  initial_translation_cov_ = this->declare_parameter<double>("initial_translation_cov", 0.05);

  angle_measurement_cov_ = this->declare_parameter<double>("angle_measurement_cov", 0.5);
  angle_process_cov_ = this->declare_parameter<double>("angle_process_cov", 0.1);
  translation_measurement_cov_ =
    this->declare_parameter<double>("translation_measurement_cov", 0.005);
  translation_process_cov_ = this->declare_parameter<double>("translation_process_cov", 0.001);

  angle_convergence_threshold_ = this->declare_parameter<float>("angle_convergence_threshold", 0.0);
  translation_convergence_threshold_ =
    this->declare_parameter<float>("translation_convergence_threshold", 0.0);

  initial_angle_cov_ = std::pow(initial_angle_cov_ * M_PI_2 / 180.0, 2);
  initial_translation_cov_ = std::pow(initial_translation_cov_, 2);

  angle_measurement_cov_ = std::pow(angle_measurement_cov_ * M_PI_2 / 180.0, 2);
  angle_process_cov_ = std::pow(angle_process_cov_ * M_PI_2 / 180.0, 2);
  translation_measurement_cov_ = std::pow(translation_measurement_cov_, 2);
  translation_process_cov_ = std::pow(translation_process_cov_, 2);

  angle_convergence_threshold_ = std::pow(angle_convergence_threshold_ * M_PI_2 / 180.0, 2);
  translation_convergence_threshold_ = std::pow(translation_convergence_threshold_, 2);

  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 10);
  inliers_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("inliers", 10);

  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_pointcloud", rclcpp::SensorDataQoS(),
    std::bind(&ExtrinsicGroundPlaneCalibrator::pointCloudCallback, this, std::placeholders::_1));

  // The service server runs in a dedicated thread
  srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  service_server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
    "extrinsic_calibration",
    std::bind(
      &ExtrinsicGroundPlaneCalibrator::requestReceivedCallback, this, std::placeholders::_1,
      std::placeholders::_2),
    rmw_qos_profile_services_default, srv_callback_group_);

  // Initialize the filter
  kalman_filter_.setA(Eigen::DiagonalMatrix<double, 6>(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
  kalman_filter_.setB(Eigen::DiagonalMatrix<double, 6>(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
  kalman_filter_.setC(Eigen::DiagonalMatrix<double, 6>(1.0, 1.0, 1.0, 1.0, 1.0, 1.0));
  kalman_filter_.setQ(Eigen::DiagonalMatrix<double, 6>(
    angle_measurement_cov_, angle_measurement_cov_, angle_measurement_cov_,
    translation_measurement_cov_, translation_measurement_cov_, translation_measurement_cov_));
  kalman_filter_.setR(Eigen::DiagonalMatrix<double, 6>(
    angle_process_cov_, angle_process_cov_, angle_process_cov_, translation_process_cov_,
    translation_process_cov_, translation_process_cov_));
}

void ExtrinsicGroundPlaneCalibrator::requestReceivedCallback(
  [[maybe_unused]] const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Request>
    request,
  const std::shared_ptr<tier4_calibration_msgs::srv::ExtrinsicCalibrator::Response> response)
{
  // This tool uses several tfs, so for consistency we take the initial calibration using lookups
  using std::chrono_literals::operator""s;

  {
    std::unique_lock<std::mutex> lock(mutex_);
    received_request_ = true;
  }

  // Loop until the calibration finishes
  while (rclcpp::ok()) {
    rclcpp::sleep_for(1s);
    std::unique_lock<std::mutex> lock(mutex_);

    if (calibration_done_) {
      break;
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000, "Waiting for the calibration to end");
  }

  tier4_calibration_msgs::msg::CalibrationResult result;
  result.transform_stamped = tf2::eigenToTransform(calibrated_base_to_lidar_transform_);
  result.transform_stamped.header.frame_id = base_frame_;
  result.transform_stamped.child_frame_id = lidar_frame_;
  result.score = 0.f;
  result.success = true;
  result.message.data =
    "Calibration succeeded with convergence criteria. However, no metric is available for this "
    "tool";

  if (overwrite_xy_yaw_) {
    result.transform_stamped = tier4_ground_plane_utils::overwriteXYYawValues(
      initial_base_to_lidar_transform_msg_, result.transform_stamped);
  }

  response->results.emplace_back(result);

  RCLCPP_INFO(this->get_logger(), "Calibration result sent");

  // Forcefully unsubscribe from the pointcloud topic
  pointcloud_sub_.reset();
}

void ExtrinsicGroundPlaneCalibrator::pointCloudCallback(
  const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  bool received_request;
  {
    std::unique_lock<std::mutex> lock(mutex_);
    received_request = received_request_;
  }

  if (lidar_frame_ != msg->header.frame_id) {
    RCLCPP_WARN(
      this->get_logger(),
      "Received pointcloud's frame does not match the expected one (received=%s vs. expected=%s)",
      msg->header.frame_id.c_str(), lidar_frame_.c_str());
    return;
  }

  header_ = msg->header;

  // Make sure we have all the required initial tfs
  if (!received_request || !checkInitialTransforms() || calibration_done_) {
    return;
  }

  // Convert the pointcloud to PCL
  pcl::PointCloud<PointType>::Ptr pointcloud(new pcl::PointCloud<PointType>);
  pcl::fromROSMsg(*msg, *pointcloud);

  // Filter the pointcloud using previous outlier models
  if (ground_plane_extractor_parameters_.remove_outliers_) {
    std::size_t original_num_points = pointcloud->size();
    for (const auto & outlier_model : outlier_models_) {
      pointcloud = tier4_ground_plane_utils::removeOutliers(
        pointcloud, outlier_model, ground_plane_extractor_parameters_.remove_outlier_tolerance_);
    }

    RCLCPP_INFO(
      this->get_logger(), "Outlier plane removal: %lu -> %lu points", original_num_points,
      pointcloud->size());
  }

  // Extract the ground plane model
  ground_plane_extractor_parameters_.initial_base_to_lidar_transform_ =
    initial_base_to_lidar_transform_;
  auto [ground_plane_result, ground_plane_model, inliers_pointcloud] =
    tier4_ground_plane_utils::extractGroundPlane(
      pointcloud, ground_plane_extractor_parameters_, outlier_models_);

  if (!ground_plane_result) {
    return;
  }

  // Obtain the model error for the initial and current calibration
  Eigen::Isometry3d initial_lidar_base_transform = initial_base_to_lidar_transform_.inverse();
  Eigen::Vector4d initial_ground_plane_model =
    tier4_ground_plane_utils::poseToPlaneModel(initial_lidar_base_transform);

  tier4_ground_plane_utils::evaluateModels(
    initial_ground_plane_model, ground_plane_model, inliers_pointcloud);

  // Publish the inliers
  sensor_msgs::msg::PointCloud2 inliers_msg;
  pcl::toROSMsg(*inliers_pointcloud, inliers_msg);
  inliers_msg.header = header_;
  inliers_pub_->publish(inliers_msg);

  // Create markers to visualize the calibration
  visualizeCalibration(ground_plane_model);

  filterGroundModelEstimation(ground_plane_model, inliers_pointcloud);

  // Obtain the final output tf and publish the lidar -> ground tfs to evaluate the calibration
  publishTf(ground_plane_model);
}

bool ExtrinsicGroundPlaneCalibrator::checkInitialTransforms()
{
  if (lidar_frame_ == "") {
    RCLCPP_ERROR(this->get_logger(), "The lidar frame can not be empty !");
    return false;
  }

  if (got_initial_transform_) {
    return true;
  }

  try {
    rclcpp::Time t = rclcpp::Time(0);
    rclcpp::Duration timeout = rclcpp::Duration::from_seconds(1.0);

    geometry_msgs::msg::Transform initial_base_to_lidar_transform_msg_ =
      tf_buffer_->lookupTransform(base_frame_, lidar_frame_, t, timeout).transform;

    initial_base_to_lidar_transform_ = tf2::transformToEigen(initial_base_to_lidar_transform_msg_);

    got_initial_transform_ = true;
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not get initial tf. %s", ex.what());
    return false;
  }

  return true;
}

void ExtrinsicGroundPlaneCalibrator::filterGroundModelEstimation(
  const Eigen::Vector4d & ground_plane_model, pcl::PointCloud<PointType>::Ptr inliers)
{
  Eigen::Isometry3d estimated_base_to_lidar_transform =
    tier4_ground_plane_utils::estimateBaseLidarTransform(
      initial_base_to_lidar_transform_, ground_plane_model);

  Eigen::Vector3d estimated_translation;
  auto estimated_rpy =
    autoware::universe_utils::getRPY(tf2::toMsg(estimated_base_to_lidar_transform).orientation);
  auto initial_rpy =
    autoware::universe_utils::getRPY(tf2::toMsg(initial_base_to_lidar_transform_).orientation);

  if (verbose_) {
    RCLCPP_INFO(
      this->get_logger(), "Initial base->lidar euler angles: roll=%.3f, pitch=%.3f, yaw=%.3f",
      initial_rpy.x, initial_rpy.y, initial_rpy.z);
    RCLCPP_INFO(
      this->get_logger(), "Estimated base->lidar euler angles: roll=%.3f, pitch=%.3f, yaw=%.3f",
      estimated_rpy.x, estimated_rpy.y, estimated_rpy.z);

    RCLCPP_INFO(
      this->get_logger(), "Initial base->lidar translation: x=%.3f, y=%.3f, z=%.3f",
      initial_base_to_lidar_transform_.translation().x(),
      initial_base_to_lidar_transform_.translation().y(),
      initial_base_to_lidar_transform_.translation().z());
    RCLCPP_INFO(
      this->get_logger(), "Estimated base->lidar translation: x=%.3f, y=%.3f, z=%.3f",
      estimated_base_to_lidar_transform.translation().x(),
      estimated_base_to_lidar_transform.translation().y(),
      estimated_base_to_lidar_transform.translation().z());
  }

  // If filtering is disabled no further processing is needed
  if (!filter_estimations_) {
    std::unique_lock<std::mutex> lock(mutex_);
    calibrated_base_to_lidar_transform_ = estimated_base_to_lidar_transform;
    calibration_done_ = true;
    return;
  }

  // Optional filtering:
  // 1) Use linear kalman filter to determine convergence in the estimation
  // 2) The liner kalman filter does not correctly filter rotations, instead use all the estimations
  // so far to calibrate once more

  // Kalman step
  Eigen::Vector<double, 6> x(
    estimated_rpy.x, estimated_rpy.y, estimated_rpy.z,
    estimated_base_to_lidar_transform.translation().x(),
    estimated_base_to_lidar_transform.translation().y(),
    estimated_base_to_lidar_transform.translation().z());
  Eigen::DiagonalMatrix<double, 6> p0(
    initial_angle_cov_, initial_angle_cov_, initial_angle_cov_, initial_translation_cov_,
    initial_translation_cov_, initial_translation_cov_);

  if (first_observation_) {
    kalman_filter_.init(x, p0);
    first_observation_ = false;
  } else {
    kalman_filter_.update(x);
  }

  // cSpell:ignore getXelement
  estimated_rpy.x = kalman_filter_.getXelement(0);
  estimated_rpy.y = kalman_filter_.getXelement(1);
  estimated_rpy.z = kalman_filter_.getXelement(2);
  estimated_translation.x() = kalman_filter_.getXelement(3);
  estimated_translation.y() = kalman_filter_.getXelement(4);
  estimated_translation.z() = kalman_filter_.getXelement(5);

  // Filtering convergence criteria
  Eigen::MatrixXd p;
  kalman_filter_.getP(p);
  Eigen::VectorXd diag = p.diagonal();
  std::array<double, 6> thresholds{
    angle_convergence_threshold_,       angle_convergence_threshold_,
    angle_convergence_threshold_,       translation_convergence_threshold_,
    translation_convergence_threshold_, translation_convergence_threshold_};

  bool converged = true;
  for (std::size_t index = 0; index < thresholds.size(); index++) {
    converged &= diag(index) < thresholds[index];
  }

  RCLCPP_INFO(
    this->get_logger(), "Filter cov: roll=%.2e, pitch=%.2e yaw=%.2e, x=%.2e, y=%.2e, z=%.2e",
    diag(0), diag(1), diag(2), diag(3), diag(4), diag(5));

  RCLCPP_INFO(
    this->get_logger(), "Convergence thresh: angle=%.2e, translation=%.2e",
    angle_convergence_threshold_, translation_convergence_threshold_);

  // Save the inliers for later refinement
  inlier_observations_.add(inliers);

  if (!converged) {
    return;
  }

  // Integrate all the inliers so far and refine the estimation
  pcl::PointCloud<PointType>::Ptr augmented_inliers(new pcl::PointCloud<PointType>);

  for (const auto & inliers : inlier_observations_.get()) {
    *augmented_inliers += *inliers;
  }

  auto [final_model, final_inliers] = tier4_ground_plane_utils::extractPlane(
    augmented_inliers, 10 * ground_plane_extractor_parameters_.max_inlier_distance_,
    ground_plane_extractor_parameters_.max_iterations_);

  RCLCPP_INFO(
    this->get_logger(),
    "Final model: a=%.3f, b=%.3f, c=%.3f, d=%.3f final inliers=%lu total.percentage=%.2f",
    final_model(0), final_model(1), final_model(2), final_model(3), final_inliers->indices.size(),
    100.f * final_inliers->indices.size() / augmented_inliers->size());

  std::unique_lock<std::mutex> lock(mutex_);
  calibrated_base_to_lidar_transform_ = tier4_ground_plane_utils::estimateBaseLidarTransform(
    initial_base_to_lidar_transform_, final_model);
  calibration_done_ = true;
}

void ExtrinsicGroundPlaneCalibrator::visualizeCalibration(
  const Eigen::Vector4d & estimated_ground_model)
{
  visualization_msgs::msg::MarkerArray markers;

  Eigen::Isometry3d initial_lidar_base_transform = initial_base_to_lidar_transform_.inverse();

  visualizePlaneModel("initial_calibration_pose", initial_lidar_base_transform, markers);

  Eigen::Vector4d fake_model =
    tier4_ground_plane_utils::poseToPlaneModel(initial_lidar_base_transform);

  visualizePlaneModel("initial_calibration_model", fake_model, markers);

  visualizePlaneModel("estimated_model", estimated_ground_model, markers);

  markers_pub_->publish(markers);
}

void ExtrinsicGroundPlaneCalibrator::visualizePlaneModel(
  const std::string & name, Eigen::Vector4d model, visualization_msgs::msg::MarkerArray & markers)
{
  visualizePlaneModel(name, tier4_ground_plane_utils::modelPlaneToPose(model), markers);
}

void ExtrinsicGroundPlaneCalibrator::visualizePlaneModel(
  const std::string & name, const Eigen::Isometry3d & lidar_base_pose,
  visualization_msgs::msg::MarkerArray & markers)
{
  std::vector<Eigen::Vector3d> corners = {
    Eigen::Vector3d(-marker_size_, -marker_size_, 0.0),
    Eigen::Vector3d(-marker_size_, marker_size_, 0.0),
    Eigen::Vector3d(marker_size_, marker_size_, 0.0),
    Eigen::Vector3d(marker_size_, -marker_size_, 0.0)};

  for (Eigen::Vector3d & corner : corners) {
    corner = lidar_base_pose * corner;
  }

  std::vector<geometry_msgs::msg::Point> corners_msg;

  for (Eigen::Vector3d & corner : corners) {
    geometry_msgs::msg::Point p;
    p.x = corner.x();
    p.y = corner.y();
    p.z = corner.z();
    corners_msg.push_back(p);
  }

  visualization_msgs::msg::Marker marker;
  marker.lifetime = rclcpp::Duration::from_seconds(1.0);
  marker.header.frame_id = lidar_frame_;
  marker.header.stamp = header_.stamp;
  marker.ns = name + "_plane";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 1.0;
  marker.scale.y = 1.0;
  marker.scale.z = 1.0;
  marker.color.a = 0.5;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;

  assert(corners_msg.size() == 4);
  marker.points.push_back(corners_msg[0]);
  marker.points.push_back(corners_msg[2]);
  marker.points.push_back(corners_msg[1]);
  marker.points.push_back(corners_msg[2]);
  marker.points.push_back(corners_msg[0]);
  marker.points.push_back(corners_msg[3]);

  markers.markers.push_back(marker);

  marker.ns = name + "_origin";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.0;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;

  geometry_msgs::msg::Point p;
  marker.points.clear();

  p.x = 0;
  p.y = 0;
  p.z = 0;
  marker.points.push_back(p);

  p.x = lidar_base_pose.translation().x();
  p.y = lidar_base_pose.translation().y();
  p.z = lidar_base_pose.translation().z();
  marker.points.push_back(p);

  markers.markers.push_back(marker);
}

void ExtrinsicGroundPlaneCalibrator::publishTf(const Eigen::Vector4d & ground_plane_model)
{
  geometry_msgs::msg::TransformStamped initial_lidar_to_base_msg =
    tf2::eigenToTransform(initial_base_to_lidar_transform_.inverse());
  initial_lidar_to_base_msg.header.stamp = header_.stamp;
  initial_lidar_to_base_msg.header.frame_id = lidar_frame_;
  initial_lidar_to_base_msg.child_frame_id = "initial_base_link";
  tf_broadcaster_.sendTransform(initial_lidar_to_base_msg);

  Eigen::Isometry3d raw_lidar_to_base_eigen =
    tier4_ground_plane_utils::modelPlaneToPose(ground_plane_model);

  geometry_msgs::msg::TransformStamped raw_lidar_to_base_msg =
    tf2::eigenToTransform(raw_lidar_to_base_eigen);

  raw_lidar_to_base_msg.header.stamp = header_.stamp;
  raw_lidar_to_base_msg.header.frame_id = lidar_frame_;
  raw_lidar_to_base_msg.child_frame_id = "ground_plane_raw";
  tf_broadcaster_.sendTransform(raw_lidar_to_base_msg);

  Eigen::Isometry3d calibrated_base_to_lidar_transform =
    calibration_done_ ? calibrated_base_to_lidar_transform_
                      : tier4_ground_plane_utils::estimateBaseLidarTransform(
                          initial_base_to_lidar_transform_, ground_plane_model);
  geometry_msgs::msg::TransformStamped calibrated_base_to_lidar_transform_msg =
    tf2::eigenToTransform(calibrated_base_to_lidar_transform);

  if (overwrite_xy_yaw_) {
    calibrated_base_to_lidar_transform_msg = tier4_ground_plane_utils::overwriteXYYawValues(
      initial_base_to_lidar_transform_msg_, calibrated_base_to_lidar_transform_msg);
  }

  geometry_msgs::msg::TransformStamped lidar_to_calibrated_base_transform_msg =
    tf2::eigenToTransform(tf2::transformToEigen(calibrated_base_to_lidar_transform_msg).inverse());
  lidar_to_calibrated_base_transform_msg.header.stamp = header_.stamp;
  lidar_to_calibrated_base_transform_msg.header.frame_id = lidar_frame_;
  lidar_to_calibrated_base_transform_msg.child_frame_id = "estimated_base_link";
  tf_broadcaster_.sendTransform(lidar_to_calibrated_base_transform_msg);
}

}  // namespace ground_plane_calibrator
