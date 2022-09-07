//
//  Copyright 2021 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.
//

#include "parameter_estimator/gear_ratio_estimator.hpp"

#include <memory>
#include <vector>

GearRatioEstimator::GearRatioEstimator(
  rclcpp::Node * node, const Params & p, const double & cov, const double ff,
  const std::vector<double> & est)
{
  error_ = 0;
  dim_x_ = est.size();
  estimated_ = Eigen::MatrixXd::Zero(dim_x_, 1);
  for (size_t i = 0; i < est.size(); i++) {
    estimated_(i, 0) = est.at(i);
  }
  covariance_ =
    (Eigen::MatrixXd::Identity(dim_x_, dim_x_) + 0.1 * Eigen::MatrixXd::Ones(dim_x_, dim_x_)) * cov;
  forgetting_factor_ = Eigen::MatrixXd::Identity(1, 1) * ff;
  params_ = p;
  debugger_ = std::make_unique<Debugger>("gear_ratio", node);
  // in estimator_base.h
  createPublisher("gear_ratio", node);
  result_statistics_ = math_utils::Statistics(dim_x_);
  error_statistics_ = math_utils::Statistics(1);
}

bool GearRatioEstimator::estimate()
{
  const auto & vel = data_.velocity;
  const auto & wheel_base = data_.wheel_base;
  const auto & wz = data_.angular_velocity;
  const auto & handle = data_.handle;
  const auto & ff = forgetting_factor_;
  auto & est = estimated_;
  auto & cov = covariance_;
  // steering = handle / gear_ratio
  // gear_ratio=(a+b*v^2-c*abs(handle))
  Eigen::MatrixXd zn = Eigen::MatrixXd::Zero(dim_x_, 1);
  Eigen::MatrixXd yn = Eigen::MatrixXd::Zero(1, 1);
  Eigen::MatrixXd th = Eigen::MatrixXd::Zero(dim_x_, 1);

  if (dim_x_ == 3) {
    yn(0, 0) = handle / std::atan2(wz * wheel_base, vel);
    zn(0, 0) = 1.0;
    zn(1, 0) = vel * vel;
    zn(2, 0) = -std::fabs(handle);
  } else if (dim_x_ == 4) {
    yn(0, 0) = handle / std::atan2(wz * wheel_base, vel);
    zn(0, 0) = 1.0;
    zn(1, 0) = vel;
    zn(2, 0) = vel * vel;
    zn(3, 0) = -std::fabs(handle);
  }
  optimization_utils::estimateByRLS(est, cov, zn, ff, yn);
  error_ = (yn - zn.transpose() * est)(0, 0);
  auto & de = debugger_->debug_values_;
  {
    th << 15.713, 0.053, 0.042;
    double gear = (zn.transpose() * est)(0, 0);
    de.data[0] = gear;
    de.data[1] = yn(0, 0);
    de.data[2] = (zn.transpose() * th)(0, 0);
  }
  // if (error > 1.0) return false;
  return true;
}

bool GearRatioEstimator::checkIsValidData()
{
  const auto & d = data_;
  debugger_->debug_values_.data[5] = 0;
  if (
    std::fabs(d.velocity) < params_.valid_min_velocity ||
    std::fabs(d.angular_velocity) < params_.valid_min_angular_velocity) {
    return false;
  }
  debugger_->debug_values_.data[5] = 1;
  return true;
}

void GearRatioEstimator::postprocessOutput()
{
  result_statistics_.value = {estimated_(0, 0), estimated_(1, 0), estimated_(2, 0)};
  error_statistics_.value = {std::fabs(error_)};
  std::vector<double> covariance = {covariance_(0, 0), covariance_(1, 1), covariance_(2, 2)};
  result_msgs_.covariance = covariance;
}

void GearRatioEstimator::publishData()
{
  auto & de = debugger_->debug_values_;
  const auto & vel = data_.velocity;
  const auto & wheel_base = data_.wheel_base;
  const auto & wz = data_.angular_velocity;
  const auto & handle = data_.handle;
  double gear =
    (1.0 * estimated_(0, 0) + vel * vel * estimated_(1, 0) - std::fabs(handle) * estimated_(2, 0));
  de.data[7] = vel * std::tan(handle / gear) / wheel_base;
  de.data[8] = wz;
  debugger_->publishDebugValue();
}
