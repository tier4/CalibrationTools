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

double double_round(const double x, const int n) { return std::round(x * pow(10, n)) / pow(10, n); }

template <typename T>
double calculateMean(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  double mean = 0;
  for (const T & t : v) {
    mean += t;
  }
  mean /= v.size();
  return mean;
}

template <typename T>
double calculateStd(const std::vector<T> & v)
{
  if (v.size() == 0) {
    return 0;
  }

  double mean = calculateMean(v);
  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

template <typename T>
double calculateStdMeanConst(const std::vector<T> & v, const double mean)
{
  if (v.size() == 0) {
    return 0;
  }

  double error = 0;
  for (const T & t : v) {
    error += pow(t - mean, 2);
  }
  return std::sqrt(error / v.size());
}

struct CompareMsgTimestamp
{
  template <typename T1>
  bool operator()(T1 const & t1, double const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2;
  }

  template <typename T2>
  bool operator()(double const & t1, T2 const & t2) const
  {
    return t1 < rclcpp::Time(t2.header.stamp).seconds();
  }

  template <typename T1, typename T2>
  bool operator()(T1 const & t1, T2 const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }

  template <typename T1>
  bool operator()(T1 const & t1, rclcpp::Time const & t2) const
  {
    return rclcpp::Time(t1.header.stamp).seconds() < t2.seconds();
  }

  template <typename T2>
  bool operator()(rclcpp::Time const & t1, T2 const & t2) const
  {
    return t1.seconds() < rclcpp::Time(t2.header.stamp).seconds();
  }
};

template <typename T>
std::vector<T> extractSubTrajectory(
  const std::vector<T> & msg_list, const rclcpp::Time t0, const rclcpp::Time t1)
{
  auto start_iter = std::lower_bound(msg_list.begin(), msg_list.end(), t0, CompareMsgTimestamp());
  auto end_iter = std::lower_bound(msg_list.begin(), msg_list.end(), t1, CompareMsgTimestamp());
  std::vector<T> msg_list_sub(start_iter, end_iter);
  return msg_list_sub;
}

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
  use_pose_with_covariance_ = declare_parameter("use_pose_with_covariance", true);
  use_twist_with_covariance_ = declare_parameter("use_twist_with_covariance", true);
  use_predefined_coef_vx_ = declare_parameter("use_predefined_coef_vx", false);
  predefined_coef_vx_ = declare_parameter("predefined_coef_vx", 1.0);
  results_path_ = declare_parameter("results_path", "test");
  imu_link_frame_ = declare_parameter("imu_link_frame", "tamagawa/imu_link");

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
  sub_pose_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "in_pose", 1, std::bind(&DeviationEstimator::callbackPose, this, _1));
  sub_twist_raw_ = create_subscription<geometry_msgs::msg::TwistStamped>(
    "in_twist_raw", 1, std::bind(&DeviationEstimator::callbackTwistRaw, this, _1));

  pub_coef_vx_ = create_publisher<std_msgs::msg::Float64>("estimated_coef_vx", 1);
  pub_bias_angvel_ =
    create_publisher<geometry_msgs::msg::Vector3>("estimated_bias_angular_velocity", 1);
  pub_stddev_vx_ = create_publisher<std_msgs::msg::Float64>("estimated_stddev_vx", 1);
  pub_stddev_angvel_ =
    create_publisher<geometry_msgs::msg::Vector3>("estimated_stddev_angular_velocity", 1);

  bias_angvel_list_.resize(3);
  bias_angvel_.resize(3);
  stddev_angvel_base_.resize(3);
  stddev_angvel_prime_base_.resize(3);
  stddev_angvel_prime_imu_.resize(3);
  tf_base2imu_ptr_ = nullptr;
  saveEstimatedParameters();

  DEBUG_INFO(this->get_logger(), "[Deviation Estimator] launch success");
}

void DeviationEstimator::callbackPoseWithCovariance(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
  // push pose_msg to queue
  if (use_pose_with_covariance_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose.pose;
    pose_buf_.push_back(pose);
    pose_all_.push_back(pose);
  }
}

void DeviationEstimator::callbackTwistWithCovarianceRaw(
  const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
{
  // push twist_msg to queue
  if (use_twist_with_covariance_ == true) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header = msg->header;
    twist.twist = msg->twist.twist;

    if (use_predefined_coef_vx_) {
      twist.twist.linear.x *= predefined_coef_vx_;
    }

    twist_all_.push_back(twist);
  }
}

void DeviationEstimator::callbackPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  // push pose_msg to queue
  if (!use_pose_with_covariance_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = msg->header;
    pose.pose = msg->pose;

    pose_buf_.push_back(pose);
    pose_all_.push_back(pose);
  }
}

void DeviationEstimator::callbackTwistRaw(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
  // push twist_msg to queue
  if (!use_twist_with_covariance_) {
    geometry_msgs::msg::TwistStamped twist;
    twist.header = msg->header;
    twist.twist = msg->twist;

    if (use_predefined_coef_vx_) {
      twist.twist.linear.x *= predefined_coef_vx_;
    }
    twist_all_.push_back(twist);
  }
}

void DeviationEstimator::timerCallback()
{
  // Update bias
  if (pose_buf_.size() != 0) {
    updateBias();
  }

  // Publish vx bias
  if (coef_vx_.second != 0) {
    std_msgs::msg::Float64 coef_vx_msg;
    coef_vx_msg.data = coef_vx_.first / coef_vx_.second;
    pub_coef_vx_->publish(coef_vx_msg);
  }

  // Publish angular velocity bias
  if (bias_angvel_[2].second != 0) {
    bias_angvel_base_.vector.x = bias_angvel_[0].first / bias_angvel_[0].second;
    bias_angvel_base_.vector.y = bias_angvel_[1].first / bias_angvel_[1].second;
    bias_angvel_base_.vector.z = bias_angvel_[2].first / bias_angvel_[2].second;

    if (tf_base2imu_ptr_ == nullptr) {
      tf_base2imu_ptr_ = std::make_shared<geometry_msgs::msg::TransformStamped>();
      getTransform(imu_link_frame_, output_frame_, tf_base2imu_ptr_);
    }
    tf2::doTransform(bias_angvel_base_, bias_angvel_imu_, *tf_base2imu_ptr_);

    pub_bias_angvel_->publish(bias_angvel_imu_.vector);
  }

  // Estimate stddev when ready
  if ((coef_vx_.second != 0) & (bias_angvel_[2].second != 0)) {
    estimateStddev();
    estimateStddevPrime();
    if (results_path_.size() > 0) {
      saveEstimatedParameters();
    }
  }
}

void DeviationEstimator::updateBias()
{
  if (twist_all_.empty()) return;
  rclcpp::Time t0_rclcpp_time = rclcpp::Time(pose_buf_.front().header.stamp);
  rclcpp::Time t1_rclcpp_time = rclcpp::Time(pose_buf_.back().header.stamp);
  if (t1_rclcpp_time <= t0_rclcpp_time) return;

  std::vector<geometry_msgs::msg::TwistStamped> twist_buf =
    extractSubTrajectory(twist_all_, t0_rclcpp_time, t1_rclcpp_time);

  double t0 = t0_rclcpp_time.seconds();
  double t1 = t1_rclcpp_time.seconds();

  // Calculate coef_vx only when the velocity is higher than the threshold
  double mean_abs_vx = 0;
  for (const auto & msg : twist_buf) {
    mean_abs_vx += abs(msg.twist.linear.x);
  }
  mean_abs_vx /= twist_buf.size();
  if (mean_abs_vx > vx_threshold_) {
    std::pair<double, double> d_pos = calculateErrorPos(pose_buf_, twist_buf, false);
    double dx = pose_buf_.back().pose.position.x - pose_buf_.front().pose.position.x;
    double dy = pose_buf_.back().pose.position.y - pose_buf_.front().pose.position.y;
    double d_coef_vx = (d_pos.first * dx + d_pos.second * dy) /
                       (d_pos.first * d_pos.first + d_pos.second * d_pos.second);

    double time_factor = (rclcpp::Time(twist_buf.back().header.stamp).seconds() -
                          rclcpp::Time(twist_buf.front().header.stamp).seconds()) /
                         (t1 - t0);
    coef_vx_.first += d_coef_vx * time_factor;
    coef_vx_.second += 1;
    coef_vx_list_.push_back(d_coef_vx);
  } else {
    DEBUG_INFO(
      this->get_logger(),
      "[Deviation Estimator] coef_vx estimation is not updated since the vehicle is not moving.");
  }

  std::vector<double> error_rpy = calculateErrorRPY(pose_buf_, twist_buf, false);
  for (int rpy = 0; rpy < 3; ++rpy) {
    bias_angvel_[rpy].first += (t1 - t0) * error_rpy[rpy];
    bias_angvel_[rpy].second += (t1 - t0) * (t1 - t0);
    bias_angvel_list_[rpy].push_back(error_rpy[rpy] / (t1 - t0));
  }

  pose_buf_.clear();
}

void DeviationEstimator::estimateStddev()
{
  double est_window_duration = 4.0;  // Hard coded

  auto duration = rclcpp::Duration(
    int(est_window_duration / 1), int((est_window_duration - est_window_duration / 1) * 1e9));

  std::vector<double> error_x_list;
  std::vector<std::vector<double>> error_rpy_lists;
  error_rpy_lists.resize(3);

  rclcpp::Time t0_rclcpp_time, t1_rclcpp_time;
  t0_rclcpp_time = rclcpp::Time(pose_all_.front().header.stamp);
  t1_rclcpp_time = t0_rclcpp_time + duration;

  // Iterate over the whole sub_trajectory every time. Calculation cost ~ O(T^2)
  while (t1_rclcpp_time < rclcpp::Time(pose_all_.back().header.stamp)) {
    std::vector<geometry_msgs::msg::PoseStamped> pose_sub_traj =
      extractSubTrajectory(pose_all_, t0_rclcpp_time, t1_rclcpp_time);

    if (
      rclcpp::Time(pose_sub_traj.back().header.stamp) >
      rclcpp::Time(pose_sub_traj.front().header.stamp)) {
      std::vector<geometry_msgs::msg::TwistStamped> twist_sub_traj = extractSubTrajectory(
        twist_all_, rclcpp::Time(pose_sub_traj.front().header.stamp),
        rclcpp::Time(pose_sub_traj.back().header.stamp));

      // calculate Error theta only if the vehicle is moving
      double x0 = pose_sub_traj.front().pose.position.x;
      double y0 = pose_sub_traj.front().pose.position.y;
      double x1 = pose_sub_traj.back().pose.position.x;
      double y1 = pose_sub_traj.back().pose.position.y;
      double distance = std::sqrt(pow(x1 - x0, 2) + pow(y1 - y0, 2));
      if (distance > est_window_duration * vx_threshold_) {
        std::vector<double> error_rpy = calculateErrorRPY(pose_sub_traj, twist_sub_traj);
        for (int rpy = 0; rpy < 3; ++rpy) {
          error_rpy_lists[rpy].push_back(error_rpy[rpy]);
        }
      }

      // calculate Error x only if the vehicle is not curving
      double mean_abs_wz = 0;
      for (auto msg : twist_sub_traj) {
        mean_abs_wz += abs(msg.twist.angular.z);
      }
      mean_abs_wz /= twist_sub_traj.size();
      if (mean_abs_wz < wz_threshold_) {
        std::pair<double, double> d_pos = calculateErrorPos(pose_sub_traj, twist_sub_traj);
        double distance_from_twist =
          std::sqrt(d_pos.first * d_pos.first + d_pos.second * d_pos.second);
        error_x_list.push_back(distance - distance_from_twist);
      }
    }

    t0_rclcpp_time += duration;
    t1_rclcpp_time += duration;
  }

  stddev_vx_ = calculateStd(error_x_list) / std::sqrt(est_window_duration);
  for (int rpy = 0; rpy < 3; ++rpy) {
    stddev_angvel_base_[rpy] = calculateStd(error_rpy_lists[rpy]) / std::sqrt(est_window_duration);
  }
}

std::pair<double, double> DeviationEstimator::calculateErrorPos(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list, const bool enable_bias)
{
  double t_prev = rclcpp::Time(twist_list.front().header.stamp).seconds();
  std::pair<double, double> d_pos;
  double yaw = getYawFromQuat(pose_list.front().pose.orientation);
  for (std::size_t i = 0; i < twist_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(twist_list[i + 1].header.stamp).seconds();
    yaw += twist_list[i].twist.angular.z * (t_cur - t_prev);
    if (enable_bias) {
      d_pos.first += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::cos(yaw) *
                     coef_vx_.first / coef_vx_.second;
      d_pos.second += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::sin(yaw) *
                      coef_vx_.first / coef_vx_.second;
    } else {
      d_pos.first += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::cos(yaw);
      d_pos.second += (t_cur - t_prev) * twist_list[i].twist.linear.x * std::sin(yaw);
    }
    t_prev = t_cur;
  }
  return d_pos;
}

std::vector<double> DeviationEstimator::calculateErrorRPY(
  const std::vector<geometry_msgs::msg::PoseStamped> & pose_list,
  const std::vector<geometry_msgs::msg::TwistStamped> & twist_list, const bool enable_bias)
{
  double roll_0 = 0, pitch_0 = 0, yaw_0 = 0;
  double roll_1 = 0, pitch_1 = 0, yaw_1 = 0;
  tf2::Quaternion q_tf_0, q_tf_1;
  tf2::fromMsg(pose_list.front().pose.orientation, q_tf_0);
  tf2::fromMsg(pose_list.back().pose.orientation, q_tf_1);
  tf2::Matrix3x3(q_tf_0).getRPY(roll_0, pitch_0, yaw_0);
  tf2::Matrix3x3(q_tf_1).getRPY(roll_1, pitch_1, yaw_1);

  double d_roll = 0, d_pitch = 0, d_yaw = 0;
  double t_prev = rclcpp::Time(twist_list.front().header.stamp).seconds();
  for (std::size_t i = 0; i < twist_list.size() - 1; ++i) {
    double t_cur = rclcpp::Time(twist_list[i + 1].header.stamp).seconds();
    if (enable_bias) {
      d_roll += (t_cur - t_prev) *
                (twist_list[i].twist.angular.x - bias_angvel_[0].first / bias_angvel_[0].second);
      d_pitch += (t_cur - t_prev) *
                 (twist_list[i].twist.angular.y - bias_angvel_[1].first / bias_angvel_[1].second);
      d_yaw += (t_cur - t_prev) *
               (twist_list[i].twist.angular.z - bias_angvel_[2].first / bias_angvel_[2].second);
    } else {
      d_roll += (t_cur - t_prev) * twist_list[i].twist.angular.x;
      d_pitch += (t_cur - t_prev) * twist_list[i].twist.angular.y;
      d_yaw += (t_cur - t_prev) * twist_list[i].twist.angular.z;
    }
    t_prev = t_cur;
  }
  double error_roll = clipRadian(-roll_1 + roll_0 + d_roll);
  double error_pitch = clipRadian(-pitch_1 + pitch_0 + d_pitch);
  double error_yaw = clipRadian(-yaw_1 + yaw_0 + d_yaw);
  std::vector<double> error_rpy = {error_roll, error_pitch, error_yaw};
  return error_rpy;
}

void DeviationEstimator::estimateStddevPrime()
{
  double stddev_coef_vx = calculateStd(coef_vx_list_);
  stddev_vx_prime_ =
    std::sqrt(pow(stddev_vx_, 2) + pow(stddev_coef_vx, 2) * pow(dx_design_, 2) / dt_design_);

  std_msgs::msg::Float64 stddev_vx_msg;
  stddev_vx_msg.data = stddev_vx_prime_;
  pub_stddev_vx_->publish(stddev_vx_msg);

  // Calculate stddev_prime (standard deviation which take the
  // bias deviation into account) for angular velocities
  for (int rpy = 0; rpy < 3; rpy++) {
    double stddev_bias_dif_rpy = calculateStdMeanConst(
      bias_angvel_list_[rpy], bias_angvel_[rpy].first / bias_angvel_[rpy].second);
    stddev_angvel_prime_base_[rpy] =
      std::sqrt(pow(stddev_angvel_base_[rpy], 2) + dt_design_ * pow(stddev_bias_dif_rpy, 2));
  }

  // base_link -> imu_link conversion
  // Here we just take the max as a rough approximation
  double stddev_angvel_imu = 0;
  for (int rpy = 0; rpy < 3; ++rpy) {
    if (stddev_angvel_imu < stddev_angvel_prime_base_[rpy]) {
      stddev_angvel_imu = stddev_angvel_prime_base_[rpy];
    }
  }
  for (int rpy = 0; rpy < 3; ++rpy) {
    stddev_angvel_prime_imu_[rpy] = stddev_angvel_imu;
  }

  geometry_msgs::msg::Vector3 stddev_angvel_imu_msg;
  stddev_angvel_imu_msg.x = stddev_angvel_prime_imu_[0];
  stddev_angvel_imu_msg.y = stddev_angvel_prime_imu_[1];
  stddev_angvel_imu_msg.z = stddev_angvel_prime_imu_[2];
  pub_stddev_angvel_->publish(stddev_angvel_imu_msg);
}

double DeviationEstimator::getYawFromQuat(const geometry_msgs::msg::Quaternion quat_msg)
{
  double r, p, y;
  tf2::Quaternion quat_t(quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w);
  tf2::Matrix3x3(quat_t).getRPY(r, p, y);
  return y;
}

double DeviationEstimator::clipRadian(const double rad)
{
  if (rad < -M_PI) {
    return rad + 2 * M_PI;
  } else if (rad >= M_PI) {
    return rad - 2 * M_PI;
  } else {
    return rad;
  }
}

/*
 * saveEstimatedParameters
 */
void DeviationEstimator::saveEstimatedParameters()
{
  std::ofstream file(results_path_);
  file << "# Results expressed in base_link" << std::endl;
  file << "# Copy the following to deviation_evaluator.param.yaml" << std::endl;
  file << "stddev_vx: " << double_round(stddev_vx_prime_, 5) << std::endl;
  file << "stddev_wz: " << double_round(stddev_angvel_prime_base_[2], 5) << std::endl;
  file << "coef_vx: " << double_round(coef_vx_.first / coef_vx_.second, 5) << std::endl;
  file << "bias_wz: " << double_round(bias_angvel_base_.vector.z, 5) << std::endl;
  file << std::endl;
  file << "# Results expressed in imu_link" << std::endl;
  file << "# Copy the following to imu_corrector.param.yaml" << std::endl;
  file << "angular_velocity_stddev_xx: " << double_round(stddev_angvel_prime_imu_[0], 5)
       << std::endl;
  file << "angular_velocity_stddev_yy: " << double_round(stddev_angvel_prime_imu_[1], 5)
       << std::endl;
  file << "angular_velocity_stddev_zz: " << double_round(stddev_angvel_prime_imu_[2], 5)
       << std::endl;
  file << "angular_velocity_offset_x: " << double_round(bias_angvel_imu_.vector.x, 6) << std::endl;
  file << "angular_velocity_offset_y: " << double_round(bias_angvel_imu_.vector.y, 6) << std::endl;
  file << "angular_velocity_offset_z: " << double_round(bias_angvel_imu_.vector.z, 6) << std::endl;

  file.close();
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
