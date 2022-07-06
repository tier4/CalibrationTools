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

#include <algorithm>
#include "time_delay_estimator/data_processor.hpp"

namespace data_processor
{
bool checkIsValidData(
  Data & input, Data & response, const Params & params, double & max_stddev,
  const double & ignore_thresh)
{
  bool is_valid_data = false;
  if (input.validation.empty()) {
    input.validation.emplace_back(input.value);
    response.validation.emplace_back(response.value);
  } else {
    const double filtered_input = math_utils::lowpassFilter(
      input.value, input.validation.back(), params.cutoff_hz_input, params.sampling_delta_time);
    input.validation.emplace_back(filtered_input);
    const double filtered_response = math_utils::lowpassFilter(
      response.value, response.validation.back(), params.cutoff_hz_input,
      params.sampling_delta_time);
    response.validation.emplace_back(filtered_response);
  }

  const auto input_val_size = static_cast<int>(input.validation.size());
  const auto response_val_size = static_cast<int>(response.validation.size());
  if (
    input_val_size > params.validation_size &&
    response_val_size > params.validation_size)
  {
    // Ignore None featured data or Too much deviation data
    double input_stddev = math_utils::getStddevFromVector(input.validation);
    double response_stddev = math_utils::getStddevFromVector(response.validation);
    max_stddev = std::max(input_stddev, response_stddev);
    if (ignore_thresh < max_stddev) {
      is_valid_data = true;
    }
  }
  if (input_val_size > params.validation_size) {
    input.validation.pop_front();
  }
  if (response_val_size > params.validation_size) {
    response.validation.pop_front();
  }
  return is_valid_data;
}
bool processInputData(Data & input, const Params & params, const int buffer)
{
  bool has_enough_input = true;
  input.raw.emplace_back(input.value);
  input.stamps.emplace_back(input.stamp);
  if (input.filtered.size() == 0) {
    input.filtered.emplace_back(input.value);
  } else {
    const double filt = math_utils::lowpassFilter(
      input.raw.back(), input.filtered.back(), params.cutoff_hz_input, params.sampling_delta_time);
    // Filtered
    input.filtered.emplace_back(filt);
    input.processed = math_utils::arrToVector(input.filtered);
    if (params.num_interpolation > 1 && input.processed.size() > 5) {
      input.processed =
        math_utils::getLinearInterpolation(input.processed, params.num_interpolation);
    }
  }

  const auto input_raw_size = static_cast<int>(input.raw.size());
  // size of raw = filtered
  if (input_raw_size < params.data_size + buffer) {
    has_enough_input = false;
  } else {
    input.stamps.pop_front();
    input.raw.pop_front();
    input.filtered.pop_front();
  }
  // fail safe
  if (input_raw_size > params.data_size + buffer) {
    RCLCPP_ERROR(
      rclcpp::get_logger("time_delay_estimator"),
      "invalid index size raw input > data_size");
    input.stamps.pop_front();
    input.raw.pop_front();
    input.filtered.pop_front();
  }
  return has_enough_input;
}

bool processResponseData(
  rclcpp::Node * node, Data & data, Data & data_dot, Data & data_2dot,
  const Params & params, const int buffer)
{
  bool has_enough_data = true;
  data.raw.emplace_back(data.value);
  data.stamps.emplace_back(data.stamp);
  if (data.filtered.size() == 0) {
    data.filtered.emplace_back(data.value);
  } else {
    const double filt = math_utils::lowpassFilter(
      data.raw.back(), data.filtered.back(), params.cutoff_hz_input, params.sampling_delta_time);
    // Filtered
    data.filtered.emplace_back(filt);
    data.processed = math_utils::arrToVector(data.filtered);
    if (params.num_interpolation > 1 && data.processed.size() > 5) {
      data.processed = math_utils::getLinearInterpolation(data.processed, params.num_interpolation);
    }
  }
  if (data.filtered.size() < 3) {
    data_dot.filtered.emplace_back(0);
    data_2dot.filtered.emplace_back(0);
    return false;
  } else {
    const double x0 = *(data.filtered.end() - 3);
    const double x1 = *(data.filtered.end() - 2);
    const double x2 = *(data.filtered.end() - 1);
    double diff =
      optimization_utils::getSecondaryCentralDifference(x0, x2, params.sampling_delta_time);
    double diff2 =
      optimization_utils::getSecondaryCentralDifference(x0, x1, x2, params.sampling_delta_time);

    auto & clk = *node->get_clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk,
      3000, "[time delay estimator] diff : " << diff << " diff2 : " << diff2);
    data_dot.filtered.emplace_back(diff);
    data_2dot.filtered.emplace_back(diff2);
    // Filtered
    data_dot.processed = math_utils::arrToVector(data_dot.filtered);
    data_2dot.processed = math_utils::arrToVector(data_2dot.filtered);
    data_dot.processed =
      math_utils::getLinearInterpolation(data_dot.processed, params.num_interpolation);
    data_2dot.processed =
      math_utils::getLinearInterpolation(data_2dot.processed, params.num_interpolation);
  }

  const auto data_raw_size = static_cast<int>(data.raw.size());
  // size of raw = filtered
  if (data_raw_size < params.data_size + buffer) {
    has_enough_data = false;
  } else {
    data.raw.pop_front();
    data.filtered.pop_front();
    data_dot.filtered.pop_front();
    data_2dot.filtered.pop_front();
    data.stamps.pop_front();
  }
  // fail safe
  if (data_raw_size > params.data_size + buffer) {
    data.raw.pop_front();
    data.filtered.pop_front();
    data_dot.filtered.pop_front();
    data_2dot.filtered.pop_front();
    data.stamps.pop_front();
    RCLCPP_ERROR(
      rclcpp::get_logger("time_delay_estimator"),
      "invalid index size raw input > data_size");
  }
  return has_enough_data;
}
}  // namespace data_processor
