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
#include "tier4_autoware_utils/geometry/geometry.hpp"

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
  time_window_ = declare_parameter("time_window", 2.0);
  add_bias_uncertainty_ = declare_parameter("add_bias_uncertainty", false);

  auto timer_control_callback = std::bind(&DeviationEstimator::timer_callback, this);
  auto period_control = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / estimation_freq_));
  timer_control_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_control_callback)>>(
    this->get_clock(), period_control, std::move(timer_control_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_control_, nullptr);

  sub_pose_with_cov_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "in_pose_with_covariance", 1,
    std::bind(&DeviationEstimator::callback_pose_with_covariance, this, _1));
  sub_wheel_odometry_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
    "in_wheel_odometry", 1, std::bind(&DeviationEstimator::callback_wheel_odometry, this, _1));
  sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
    "in_imu", 1, std::bind(&DeviationEstimator::callback_imu, this, _1));
  pub_coef_vx_ = create_publisher<std_msgs::msg::Float64>("estimated_coef_vx", 1);
  pub_bias_angvel_ =
    create_publisher<geometry_msgs::msg::Vector3>("estimated_bias_angular_velocity", 1);
  pub_stddev_vx_ = create_publisher<std_msgs::msg::Float64>("estimated_stddev_vx", 1);
  pub_stddev_angvel_ =
    create_publisher<geometry_msgs::msg::Vector3>("estimated_stddev_angular_velocity", 1);

  save_estimated_parameters(
    results_path_, 0.2, 0.03, 0.0, 0.0, geometry_msgs::msg::Vector3{},
    geometry_msgs::msg::Vector3{});

  gyro_bias_module_ = std::make_unique<GyroBiasModule>();
  vel_coef_module_ = std::make_unique<VelocityCoefModule>();
  transform_listener_ = std::make_shared<tier4_autoware_utils::TransformListener>(this);

  DEBUG_INFO(this->get_logger(), "[Deviation Estimator] launch success");
}

void DeviationEstimator::callback_pose_with_covariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // push pose_msg to queue
  geometry_msgs::msg::PoseStamped pose;
  pose.header = msg->header;
  pose.pose = msg->pose.pose;
  pose_buf_.push_back(pose);
  pose_all_.push_back(pose);
}

void DeviationEstimator::callback_wheel_odometry(
  const geometry_msgs::msg::TwistWithCovarianceStamped::ConstSharedPtr wheel_odometry_msg_ptr)
{
  tier4_debug_msgs::msg::Float64Stamped vx;
  vx.stamp = wheel_odometry_msg_ptr->header.stamp;
  vx.data = wheel_odometry_msg_ptr->twist.twist.linear.x;

  if (use_predefined_coef_vx_) {
    vx.data *= predefined_coef_vx_;
  }

  vx_all_.push_back(vx);
}

void DeviationEstimator::callback_imu(const sensor_msgs::msg::Imu::ConstSharedPtr imu_msg_ptr)
{
  tf_imu2base_ptr_ = transform_listener_->getLatestTransform(imu_msg_ptr->header.frame_id, output_frame_);
  if (!tf_imu2base_ptr_) {
    RCLCPP_ERROR(
      this->get_logger(), "Please publish TF %s to %s", output_frame_.c_str(),
      (imu_msg_ptr->header.frame_id).c_str());
    return;
  }

  geometry_msgs::msg::Vector3Stamped gyro;
  gyro.header.stamp = imu_msg_ptr->header.stamp;
  gyro.vector = transform_vector3(imu_msg_ptr->angular_velocity, *tf_imu2base_ptr_);

  gyro_all_.push_back(gyro);
}

void DeviationEstimator::timer_callback()
{
  if (gyro_all_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No IMU data");
    return;
  }
  if (vx_all_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No wheel odometry");
    return;
  }
  if (pose_all_.empty()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "No pose");
    return;
  }
  if (pose_buf_.size() == 0) return;
  rclcpp::Time t0_rclcpp_time = rclcpp::Time(pose_buf_.front().header.stamp);
  rclcpp::Time t1_rclcpp_time = rclcpp::Time(pose_buf_.back().header.stamp);
  if (t1_rclcpp_time <= t0_rclcpp_time) return;

  std::vector<tier4_debug_msgs::msg::Float64Stamped> vx_buf =
    extract_sub_trajectory(vx_all_, t0_rclcpp_time, t1_rclcpp_time);
  std::vector<geometry_msgs::msg::Vector3Stamped> gyro_buf =
    extract_sub_trajectory(gyro_all_, t0_rclcpp_time, t1_rclcpp_time);

  double t0 = t0_rclcpp_time.seconds();
  double t1 = t1_rclcpp_time.seconds();

  if (get_mean_abs_vx(vx_buf) > vx_threshold_) {
    vel_coef_module_->update_coef(pose_buf_, vx_buf, gyro_buf, t1 - t0);
  } else {
    DEBUG_INFO(
      this->get_logger(),
      "[Deviation Estimator] coef_vx estimation is not updated since the vehicle is not moving.");
  }
  gyro_bias_module_->update_bias(pose_buf_, gyro_buf, t1 - t0);
  pose_buf_.clear();

  if (vel_coef_module_->empty() | gyro_bias_module_->empty()) return;
  double stddev_vx = estimate_stddev_velocity(pose_all_, vx_all_, gyro_all_, time_window_);
  auto stddev_angvel_base = estimate_stddev_angular_velocity(pose_all_, gyro_all_, time_window_);
  if (add_bias_uncertainty_) {
    stddev_vx = add_bias_uncertainty_on_velocity(stddev_vx, vel_coef_module_->get_coef_std());
    stddev_angvel_base = add_bias_uncertainty_on_angular_velocity(
      stddev_angvel_base, gyro_bias_module_->get_bias_std());
  }

  // publish messages
  std_msgs::msg::Float64 coef_vx_msg;
  coef_vx_msg.data = vel_coef_module_->get_coef();
  pub_coef_vx_->publish(coef_vx_msg);

  // geometry_msgs::msg::Vector3Stamped bias_angvel_base;
  // bias_angvel_base.header.stamp = this->now();
  // bias_angvel_base.vector = gyro_bias_module_->get_bias_base_link();
  geometry_msgs::msg::TransformStamped tf_base2imu = inverse_transform(*tf_imu2base_ptr_);
  const geometry_msgs::msg::Vector3 bias_angvel_imu = transform_vector3(gyro_bias_module_->get_bias_base_link(), tf_base2imu);
  pub_bias_angvel_->publish(bias_angvel_imu);

  std_msgs::msg::Float64 stddev_vx_msg;
  stddev_vx_msg.data = stddev_vx;
  pub_stddev_vx_->publish(stddev_vx_msg);

  double stddev_angvel_imu = 0.0;
  stddev_angvel_imu = std::max(stddev_angvel_imu, stddev_angvel_base.x);
  stddev_angvel_imu = std::max(stddev_angvel_imu, stddev_angvel_base.y);
  stddev_angvel_imu = std::max(stddev_angvel_imu, stddev_angvel_base.z);
  geometry_msgs::msg::Vector3 stddev_angvel_imu_msg =
    tier4_autoware_utils::createVector3(stddev_angvel_imu, stddev_angvel_imu, stddev_angvel_imu);
  pub_stddev_angvel_->publish(stddev_angvel_imu_msg);

  if (!results_path_.empty()) {
    save_estimated_parameters(
      results_path_, stddev_vx, stddev_angvel_base.z, vel_coef_module_->get_coef(),
      gyro_bias_module_->get_bias_base_link().z, stddev_angvel_imu_msg, bias_angvel_imu);
  }
}

double DeviationEstimator::estimate_stddev_velocity(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<tier4_debug_msgs::msg::Float64Stamped> & vx_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list, const double t_window) const
{
  auto duration = rclcpp::Duration::from_seconds(t_window);
  std::vector<double> delta_x_list;
  const rclcpp::Time t_pose_start = rclcpp::Time(pose_list.front().header.stamp);
  const rclcpp::Time t_pose_end = rclcpp::Time(pose_list.back().header.stamp);
  // Iterate over the whole sub_trajectory every time. Calculation cost ~ O(T^2)
  for (int i = 0; i < (t_pose_end - t_pose_start).seconds() / duration.seconds() - 1; ++i) {
    const rclcpp::Time t0_rclcpp_time = t_pose_start + duration * i;
    const rclcpp::Time t1_rclcpp_time = t_pose_start + duration * (i + 1);
    const std::vector<geometry_msgs::msg::PoseStamped> pose_sub_traj =
      extract_sub_trajectory(pose_list, t0_rclcpp_time, t1_rclcpp_time);

    const auto t1_pose = rclcpp::Time(pose_sub_traj.back().header.stamp);
    const auto t0_pose = rclcpp::Time(pose_sub_traj.front().header.stamp);
    if (t0_pose > t1_pose) continue;

    const std::vector<tier4_debug_msgs::msg::Float64Stamped> vx_sub_traj =
      extract_sub_trajectory(vx_list, t0_pose, t1_pose);
    const std::vector<geometry_msgs::msg::Vector3Stamped> gyro_sub_traj =
      extract_sub_trajectory(gyro_list, t0_pose, t1_pose);
    const size_t n_twist = vx_sub_traj.size();

    if (get_mean_abs_vx(vx_sub_traj) < vx_threshold_) continue;
    if (get_mean_abs_wz(gyro_sub_traj) > wz_threshold_) continue;

    const double distance =
      norm_xy(pose_sub_traj.front().pose.position, pose_sub_traj.back().pose.position);
    const auto d_pos = integrate_position(
      vx_sub_traj, gyro_sub_traj, vel_coef_module_->get_coef(),
      tf2::getYaw(pose_sub_traj.front().pose.orientation));

    const double distance_from_twist = std::sqrt(d_pos.x * d_pos.x + d_pos.y * d_pos.y);
    const double delta = std::sqrt(n_twist / t_window) * (distance - distance_from_twist);
    delta_x_list.push_back(delta);
  }
  return calculate_std(delta_x_list) / std::sqrt(t_window);
}

geometry_msgs::msg::Vector3 DeviationEstimator::estimate_stddev_angular_velocity(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::Vector3Stamped> & gyro_list, const double t_window) const
{
  auto duration = rclcpp::Duration::from_seconds(t_window);
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
      extract_sub_trajectory(pose_list, t0_rclcpp_time, t1_rclcpp_time);

    const auto t1_pose = rclcpp::Time(pose_sub_traj.back().header.stamp);
    const auto t0_pose = rclcpp::Time(pose_sub_traj.front().header.stamp);
    if (t0_pose > t1_pose) continue;

    const std::vector<geometry_msgs::msg::Vector3Stamped> gyro_sub_traj =
      extract_sub_trajectory(gyro_list, t0_pose, t1_pose);
    const size_t n_twist = gyro_sub_traj.size();

    const auto error_rpy =
      calculate_error_rpy(pose_sub_traj, gyro_sub_traj, gyro_bias_module_->get_bias_base_link());
    delta_wx_list.push_back(std::sqrt(n_twist / t_window) * error_rpy.x);
    delta_wy_list.push_back(std::sqrt(n_twist / t_window) * error_rpy.y);
    delta_wz_list.push_back(std::sqrt(n_twist / t_window) * error_rpy.z);
  }

  geometry_msgs::msg::Vector3 stddev_angvel_base = tier4_autoware_utils::createVector3(
    calculate_std(delta_wx_list) / std::sqrt(t_window),
    calculate_std(delta_wy_list) / std::sqrt(t_window),
    calculate_std(delta_wz_list) / std::sqrt(t_window));
  return stddev_angvel_base;
}

double DeviationEstimator::add_bias_uncertainty_on_velocity(
  const double stddev_vx, const double stddev_coef_vx) const
{
  const double stddev_vx_prime =
    std::sqrt(pow(stddev_vx, 2) + pow(stddev_coef_vx, 2) * pow(dx_design_, 2) / dt_design_);
  return stddev_vx_prime;
}

geometry_msgs::msg::Vector3 DeviationEstimator::add_bias_uncertainty_on_angular_velocity(
  const geometry_msgs::msg::Vector3 stddev_angvel_base,
  const geometry_msgs::msg::Vector3 stddev_angvel_bias_base) const
{
  geometry_msgs::msg::Vector3 stddev_angvel_prime_base = tier4_autoware_utils::createVector3(
    std::sqrt(pow(stddev_angvel_base.x, 2) + dt_design_ * pow(stddev_angvel_bias_base.x, 2)),
    std::sqrt(pow(stddev_angvel_base.y, 2) + dt_design_ * pow(stddev_angvel_bias_base.y, 2)),
    std::sqrt(pow(stddev_angvel_base.z, 2) + dt_design_ * pow(stddev_angvel_bias_base.z, 2)));
  return stddev_angvel_prime_base;
}
