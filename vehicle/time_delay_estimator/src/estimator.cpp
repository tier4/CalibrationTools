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

#include "estimator_utils/math_utils.hpp"
#include "time_delay_estimator/time_delay_estimator.hpp"

#include <limits>
#include <string>
#include <vector>

TimeDelayEstimator::DetectionResult TimeDelayEstimator::estimateDelayByLeastSquared(
  const std::vector<double> & x_dot, const std::vector<double> & x, const std::vector<double> & u,
  const Params & params)
{
  int num_sample = static_cast<int>(u.size() * (1.0 - params.valid_delay_index_ratio));
  int maximum_delay = static_cast<int>(u.size() * params.valid_delay_index_ratio);
  std::vector<double> resample_y = {x.end() - num_sample, x.end()};
  std::vector<double> resample_y_dot = {x_dot.end() - num_sample, x_dot.end()};
  std::vector<double> resample_x;
  double min_error = std::numeric_limits<double>::max();

  int min_error_index = 0;
  for (int d = 0; d < maximum_delay; d++) {
    // assume std::vector(old,....,new)
    resample_x = {u.end() - num_sample - d, u.end() - d};
    double error_norm = optimization_utils::getLeastSquaredError(
      resample_y_dot, resample_y, resample_x, ls_estimator_.w);
    if (optimization_utils::change_abs_min(min_error, error_norm)) {
      min_error = error_norm;
      min_error_index = d;
    }
  }
  ls_estimator_.mae = min_error;
  ls_estimator_.estimated_delay_index = min_error_index;
  ls_estimator_.time_delay = static_cast<double>(min_error_index) * params.sampling_delta_time /
                             static_cast<double>(params.num_interpolation);
  const double valid_mae_threshold = 0.1;
  if (ls_estimator_.mae < valid_mae_threshold) {
    return DetectionResult::BELOW_THRESH;
  }
  return DetectionResult::DETECTED;
}

TimeDelayEstimator::DetectionResult TimeDelayEstimator::estimateDelayByLeastSquared(
  const std::vector<double> & x2dot, const std::vector<double> & x_dot,
  const std::vector<double> & x, const std::vector<double> & u, const Params & params)
{
  int num_sample = static_cast<int>(u.size() * (1.0 - params.valid_delay_index_ratio));
  int maximum_delay = static_cast<int>(u.size() * params.valid_delay_index_ratio);
  std::vector<double> resample_y = {x.end() - num_sample, x.end()};
  std::vector<double> resample_y_dot = {x_dot.end() - num_sample, x_dot.end()};
  std::vector<double> resample_y2dot = {x2dot.end() - num_sample, x2dot.end()};
  std::vector<double> resample_x;
  double min_error = std::numeric_limits<double>::max();
  int min_error_index = 0;
  min_error = std::numeric_limits<double>::max();
  for (int d = 0; d < maximum_delay; d++) {
    //  assume std::vector(old,....,new)
    resample_x = {u.end() - num_sample - d, u.end() - d};
    double error_norm = optimization_utils::getLeastSquaredError(
      resample_y2dot, resample_y_dot, resample_y, resample_x, ls2_estimator_.w);
    if (optimization_utils::change_abs_min(min_error, error_norm)) {
      min_error = error_norm;
      min_error_index = d;
    }
  }
  ls2_estimator_.mae = min_error;
  ls2_estimator_.estimated_delay_index = min_error_index;
  ls2_estimator_.time_delay = static_cast<double>(min_error_index) * params.sampling_delta_time /
                              static_cast<double>(params.num_interpolation);
  const double valid_mae_threshold = 0.1;
  if (ls2_estimator_.mae < valid_mae_threshold) {
    return DetectionResult::DETECTED;
  }
  return DetectionResult::BELOW_THRESH;
}

TimeDelayEstimator::DetectionResult TimeDelayEstimator::estimateDelayByCrossCorrelation(
  rclcpp::Node * node, const std::vector<double> & input, const std::vector<double> & response,
  Estimator & cc_estimator, std::string name, const Params & params)
{
  auto & cross_corr = cc_estimator.cross_correlation;
  cross_corr = math_utils::calcCrossCorrelationCoefficient(
    input, response, weights_for_data_, params.valid_delay_index_ratio);
  auto & peak_index = cc_estimator.estimated_delay_index;
  peak_index = math_utils::getMaximumIndexFromVector(cross_corr);
  auto & peak_corr = cc_estimator.peak_correlation;
  peak_corr = cross_corr[peak_index];
  if (peak_corr < params.valid_peak_cross_correlation_threshold) {
    auto & clk = *node->get_clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk, 5000,
      name << "[time_delay_estimator] : correlation peak less than thresh: " << peak_corr);
    return DetectionResult::BELOW_THRESH;
  }
  cc_estimator.mae = math_utils::calcMAE(input, response, cc_estimator.estimated_delay_index);
  cc_estimator.time_delay = static_cast<double>(peak_index) * params.sampling_delta_time /
                            static_cast<double>(params.num_interpolation);
  return DetectionResult::DETECTED;
}
