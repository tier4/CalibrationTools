// Copyright 2018-2019 Autoware Foundation
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

#include "deviation_estimator/deviation_estimator.hpp"

#include "deviation_estimator/utils.hpp"
#include "rclcpp/logging.hpp"

#include <algorithm>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// clang-format off
#define PRINT_MAT(X) std::cout << #X << ":\n" << X << std::endl << std::endl
#define DEBUG_INFO(...) {if (show_debug_info_) {RCLCPP_INFO(__VA_ARGS__);}}
#define DEBUG_PRINT_MAT(X) {if (show_debug_info_) {std::cout << #X << ": " << X << std::endl;}}

// clang-format on
using std::placeholders::_1;

DeviationEstimator::DeviationEstimator(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: rclcpp::Node(node_name, node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  output_frame_(declare_parameter("base_link", "base_link"))
{
  show_debug_info_ = declare_parameter("show_debug_info", false);
  dt_design_ = declare_parameter("dt_design", 10.0);
  dx_design_ = declare_parameter("dx_design", 30.0);
  vx_threshold_ = declare_parameter("vx_threshold", 1.5);
  wz_threshold_ = declare_parameter("wz_threshold", 0.01);
  estimation_freq_ = declare_parameter("estimation_freq", 0.5);
  use_predefined_coef_vx_ = declare_parameter("use_predefined_coef_vx", false);
  predefined_coef_vx_ = declare_parameter("predefined_coef_vx", 1.0);
  results_path_ = declare_parameter("results_path", "test");
  imu_link_frame_ = declare_parameter("imu_link_frame", "tamagawa/imu_link");
  time_window_ = declare_parameter("time_window", 2.0);
  add_bias_uncertainty_ = declare_parameter("add_bias_uncertainty", false);

  auto timer_control_callback = std::bind(&DeviationEstimator::timerCallback, this);
  auto period_control = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / estimation_freq_));
  timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_control_callback)>>(
    this->get_clock(), period_control, std::move(timer_control_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_control_, nullptr);

  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&DeviationEstimator::callbackPoseWithCovariance, this, _1));
  sub_twist_with_cov_raw_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_twist_with_covariance_raw", 1,
    std::bind(&DeviationEstimator::callbackTwistWithCovarianceRaw, this, _1));
  pub_coef_vx_ = create_publisher<std_msgs::msg::Float64>("estimated_coef_vx", 1);
  pub_bias_angvel_ =
    create_publisher<geometry_msgs::msg::Vector3>("estimated_bias_angular_velocity", 1);
  pub_stddev_vx_ = create_publisher<std_msgs::msg::Float64>("estimated_stddev_vx", 1);
  pub_stddev_angvel_ =
    create_publisher<geometry_msgs::msg::Vector3>("estimated_stddev_angular_velocity", 1);

  tf_base2imu_ptr_ = nullptr;
  saveEstimatedParameters(
    results_path_, 0.2, 0.03, 0.0, 0.0, geometry_msgs::msg::Vector3{},
    geometry_msgs::msg::Vector3{});

  gyro_bias_module_ = std::make_unique<GyroBiasModule>();
  vel_coef_module_ = std::make_unique<VelocityCoefModule>();

  DEBUG_INFO(this->get_logger(), "[Deviation Estimator] launch success");
}

void DeviationEstimator::callbackPoseWithCovariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // push pose_msg to queue
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  pose_buf_.push_back(pose);
  pose_all_.push_back(pose);
}

void DeviationEstimator::callbackTwistWithCovarianceRaw(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  static std::vector<double> tmp;
  static std::vector<double> stamps;

  // push twist_msg to queue
  geometry_msgs::msg::TwistStamped twist;
  twist.header = msg->header;
  twist.twist = msg->twist.twist;

  if (use_predefined_coef_vx_) {
    twist.twist.linear.x *= predefined_coef_vx_;
  }

  twist_all_.push_back(twist);
}

void DeviationEstimator::timerCallback()
{
  if (twist_all_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No twist");
    return;
  }
  if (pose_all_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No pose");
    return;
  }
  if (pose_buf_.size() == 0) return;
  updateBias(pose_buf_, twist_all_);
  pose_buf_.clear();

  if (vel_coef_module_->empty() | gyro_bias_module_->empty()) return;
  double stddev_vx = estimateStddevVelocity(pose_all_, twist_all_, time_window_);
  auto stddev_angvel_base = estimateStddevAngularVelocity(pose_all_, twist_all_, time_window_);
  if (add_bias_uncertainty_)
  {
    stddev_vx =
      addBiasUncertaintyOnVelocity(stddev_vx, vel_coef_module_->get_coef_std());
    stddev_angvel_base =
      addBiasUncertaintyOnAngularVelocity(stddev_angvel_base, gyro_bias_module_->get_bias_std());
  }

  // publish messages
  std_msgs::msg::Float64 coef_vx_msg;
  coef_vx_msg.data = vel_coef_module_->get_coef();
  pub_coef_vx_->publish(coef_vx_msg);

  geometry_msgs::msg::Vector3Stamped bias_angvel_base;
  bias_angvel_base.header.stamp = this->now();
  bias_angvel_base.vector = gyro_bias_module_->get_bias_base_link();

  if (tf_base2imu_ptr_ == nullptr) {
    tf_base2imu_ptr_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
    getTransform(imu_link_frame_, output_frame_, tf_base2imu_ptr_);
  }
  geometry_msgs::msg::Vector3Stamped bias_angvel_imu;
  tf2::doTransform(bias_angvel_base, bias_angvel_imu, *tf_base2imu_ptr_);
  pub_bias_angvel_->publish(bias_angvel_imu.vector);

  std_msgs::msg::Float64 stddev_vx_msg;
  stddev_vx_msg.data = stddev_vx;
  pub_stddev_vx_->publish(stddev_vx_msg);

  double stddev_angvel_imu = 0.0;
  stddev_angvel_imu = std::max(stddev_angvel_imu, stddev_angvel_base.x);
  stddev_angvel_imu = std::max(stddev_angvel_imu, stddev_angvel_base.y);
  stddev_angvel_imu = std::max(stddev_angvel_imu, stddev_angvel_base.z);
  geometry_msgs::msg::Vector3 stddev_angvel_imu_msg;
  stddev_angvel_imu_msg.x = stddev_angvel_imu;
  stddev_angvel_imu_msg.y = stddev_angvel_imu;
  stddev_angvel_imu_msg.z = stddev_angvel_imu;
  pub_stddev_angvel_->publish(stddev_angvel_imu_msg);

  if (results_path_.size() > 0) {
    saveEstimatedParameters(
      results_path_, stddev_vx, stddev_angvel_base.z, vel_coef_module_->get_coef(),
      bias_angvel_base.vector.z, stddev_angvel_imu_msg, bias_angvel_imu.vector);
  };
}

void DeviationEstimator::updateBias(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_buf,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_all)
{
  rclcpp::Time t0_rclcpp_time = rclcpp::Time(pose_buf.front().header.stamp);
  rclcpp::Time t1_rclcpp_time = rclcpp::Time(pose_buf.back().header.stamp);
  if (t1_rclcpp_time <= t0_rclcpp_time) return;

  std::vector<geometry_msgs::msg::TwistStamped> twist_buf =
    extractSubTrajectory(twist_all, t0_rclcpp_time, t1_rclcpp_time);

  double t0 = t0_rclcpp_time.seconds();
  double t1 = t1_rclcpp_time.seconds();

  if (getMeanAbsVx(twist_buf) > vx_threshold_) {
    vel_coef_module_->update_coef(pose_buf, twist_buf, t1 - t0);
  } else {
    DEBUG_INFO(
      this->get_logger(),
      "[Deviation Estimator] coef_vx estimation is not updated since the vehicle is not moving.");
  }
  gyro_bias_module_->update_bias(pose_buf, twist_buf, t1 - t0);
}

double DeviationEstimator::estimateStddevVelocity(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list, const double T_window) const
{
  auto duration = rclcpp::Duration::from_seconds(T_window);
  std::vector<double> delta_x_list;
  const rclcpp::Time t_pose_start = rclcpp::Time(pose_list.front().header.stamp);
  const rclcpp::Time t_pose_end = rclcpp::Time(pose_list.back().header.stamp);
  // Iterate over the whole sub_trajectory every time. Calculation cost ~ O(T^2)
  for (int i = 0; i < (t_pose_end - t_pose_start).seconds() / duration.seconds() - 1; ++i) {
    const rclcpp::Time t0_rclcpp_time = t_pose_start + duration * i;
    const rclcpp::Time t1_rclcpp_time = t_pose_start + duration * (i + 1);
    const std::vector<geometry_msgs::msg::PoseStamped> pose_sub_traj =
      extractSubTrajectory(pose_list, t0_rclcpp_time, t1_rclcpp_time);

    const auto t1_pose = rclcpp::Time(pose_sub_traj.back().header.stamp);
    const auto t0_pose = rclcpp::Time(pose_sub_traj.front().header.stamp);
    if (t0_pose > t1_pose) continue;

    const std::vector<geometry_msgs::msg::TwistStamped> twist_sub_traj =
      extractSubTrajectory(twist_list, t0_pose, t1_pose);
    const size_t N_twist = twist_sub_traj.size();

    if (getMeanAbsVx(twist_sub_traj) < vx_threshold_) continue;
    if (getMeanAbsWz(twist_sub_traj) > wz_threshold_) continue;

    const double distance =
      norm_xy(pose_sub_traj.front().pose.position, pose_sub_traj.back().pose.position);
    const auto d_pos = calculateErrorPos(pose_sub_traj, twist_sub_traj, vel_coef_module_->get_coef());

    const double distance_from_twist = std::sqrt(d_pos.x * d_pos.x + d_pos.y * d_pos.y);
    const double delta = std::sqrt(N_twist / T_window) * (distance - distance_from_twist);
    delta_x_list.push_back(delta);
  }
  return calculateStd(delta_x_list) / std::sqrt(T_window);
}

geometry_msgs::msg::Vector3 DeviationEstimator::estimateStddevAngularVelocity(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list, const double T_window) const
{
  auto duration = rclcpp::Duration::from_seconds(T_window);
  std::vector<double> delta_wx_list;
  std::vector<double> delta_wy_list;
  std::vector<double> delta_wz_list;

  const rclcpp::Time t_pose_start = rclcpp::Time(pose_list.front().header.stamp);
  const rclcpp::Time t_pose_end = rclcpp::Time(pose_list.back().header.stamp);
  // Iterate over the whole sub_trajectory every time. Calculation cost ~ O(T^2)
  for (int i = 0; i < (t_pose_end - t_pose_start).seconds() / duration.seconds() - 1; ++i) {
    const rclcpp::Time t0_rclcpp_time = t_pose_start + duration * i;
    const rclcpp::Time t1_rclcpp_time = t_pose_start + duration * (i + 1);
    const std::vector<geometry_msgs::msg::PoseStamped> pose_sub_traj =
      extractSubTrajectory(pose_list, t0_rclcpp_time, t1_rclcpp_time);

    const auto t1_pose = rclcpp::Time(pose_sub_traj.back().header.stamp);
    const auto t0_pose = rclcpp::Time(pose_sub_traj.front().header.stamp);
    if (t0_pose > t1_pose) continue;

    const std::vector<geometry_msgs::msg::TwistStamped> twist_sub_traj =
      extractSubTrajectory(twist_list, t0_pose, t1_pose);
    const size_t N_twist = twist_sub_traj.size();

    const auto error_rpy = calculateErrorRPY(pose_sub_traj, twist_sub_traj, gyro_bias_module_->get_bias_base_link());
    delta_wx_list.push_back(std::sqrt(N_twist / T_window) * error_rpy.x);
    delta_wy_list.push_back(std::sqrt(N_twist / T_window) * error_rpy.y);
    delta_wz_list.push_back(std::sqrt(N_twist / T_window) * error_rpy.z);
  }

  geometry_msgs::msg::Vector3 stddev_angvel_base;
  stddev_angvel_base.x = calculateStd(delta_wx_list) / std::sqrt(T_window);
  stddev_angvel_base.y = calculateStd(delta_wy_list) / std::sqrt(T_window);
  stddev_angvel_base.z = calculateStd(delta_wz_list) / std::sqrt(T_window);
  return stddev_angvel_base;
}

double DeviationEstimator::addBiasUncertaintyOnVelocity(
  const double stddev_vx, const double stddev_coef_vx) const
{
  const double stddev_vx_prime =
    std::sqrt(pow(stddev_vx, 2) + pow(stddev_coef_vx, 2) * pow(dx_design_, 2) / dt_design_);
  return stddev_vx_prime;
}

geometry_msgs::msg::Vector3 DeviationEstimator::addBiasUncertaintyOnAngularVelocity(
  const geometry_msgs::msg::Vector3 stddev_angvel_base,
  const geometry_msgs::msg::Vector3 stddev_angvel_bias_base) const
{
  geometry_msgs::msg::Vector3 stddev_angvel_prime_base;
  stddev_angvel_prime_base.x =
    std::sqrt(pow(stddev_angvel_base.x, 2) + dt_design_ * pow(stddev_angvel_bias_base.x, 2));
  stddev_angvel_prime_base.y =
    std::sqrt(pow(stddev_angvel_base.y, 2) + dt_design_ * pow(stddev_angvel_bias_base.y, 2));
  stddev_angvel_prime_base.z =
    std::sqrt(pow(stddev_angvel_base.z, 2) + dt_design_ * pow(stddev_angvel_bias_base.z, 2));

  return stddev_angvel_prime_base;
}

bool DeviationEstimator::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::msg::TransformStamped::SharedPtr transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = this->get_clock()->now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}
