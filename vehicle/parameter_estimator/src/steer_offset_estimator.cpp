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

#include "parameter_estimator/steer_offset_estimator.hpp"

#include <memory>
#include <vector>
SteerOffsetEstimator::SteerOffsetEstimator(
  rclcpp::Node * node, const Params & p, double cov, const double ff, const double est)
{
  error_ = 0;
  covariance_ = cov;
  forgetting_factor_ = ff;
  estimated_ = est;
  params_ = p;
  debugger_ = std::make_unique<Debugger>("steer_offset", node);
  createPublisher("steer_offset", node);
  result_statistics_ = math_utils::Statistics(1);
  error_statistics_ = math_utils::Statistics(1);
}

bool SteerOffsetEstimator::estimate()
{
  // use following approximation: tan(a+b) = tan(a) + tan(b) = a + b
  const auto & vel = data_.velocity;
  const auto & wz = data_.angular_velocity;
  const auto & wheel_base = data_.wheel_base;
  const auto & steer = data_.steer;
  const double phi = vel / wheel_base;
  const double yn = wz - phi * steer;
  auto & cov = covariance_;
  const auto & ff = forgetting_factor_;
  auto & est = estimated_;
  optimization_utils::estimateByRLS(est, cov, phi, ff, yn);
  error_ = yn - phi * est;
  // if (error > 1.0) return false;
  return true;
}

bool SteerOffsetEstimator::checkIsValidData()
{
  const auto & d = data_;
  debugger_->debug_values_.data[5] = 0;
  if (
    std::fabs(d.velocity) < params_.valid_min_velocity ||
    std::fabs(d.steer) > params_.valid_max_steer_rad) {
    return false;
  }
  debugger_->debug_values_.data[5] = 1;
  return true;
}

void SteerOffsetEstimator::postprocessOutput()
{
  result_statistics_.value = {estimated_};
  error_statistics_.value = {std::fabs(error_)};
  std::vector<double> covariance = {covariance_};
  result_msgs_.covariance = covariance;
}

void SteerOffsetEstimator::publishData() { debugger_->publishDebugValue(); }
