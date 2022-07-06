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

#include <string>
#include <memory>
#include "time_delay_estimator/time_delay_estimator.hpp"
#include "time_delay_estimator/data_processor.hpp"
#include "time_delay_estimator/parameters.hpp"

TimeDelayEstimator::TimeDelayEstimator(
  rclcpp::Node * node, const Params & params, const std::string & name, int total_data_size,
  bool use_weight_for_cross_correlation)
{
  params_ = params;
  debugger_ = std::make_unique<Debugger>(node, name);
  this->name_ = name;
  ignore_thresh_ = node->declare_parameter<double>(name + "/min_stddev_threshold", 0.005);
  weights_for_data_.clear();
  for (size_t i = total_data_size; i > 0; i--) {
    if (use_weight_for_cross_correlation) {
      weights_for_data_.emplace_back(static_cast<double>(i));
    } else {
      weights_for_data_.emplace_back(static_cast<double>(1.0));
    }
  }
}

void TimeDelayEstimator::resetEstimator()
{
  cc_estimator_ = Estimator();
  ls_estimator_ = Estimator();
  ls2_estimator_ = Estimator();
  loop_count_ = 0;
  has_enough_input_ = false;
  has_enough_response_ = false;
  max_current_stddev_ = 0;
  input_ = {};
  response_ = {};
  response_dot_ = {};
  response_2dot_ = {};
  time_delay_ = tier4_calibration_msgs::msg::TimeDelay();
}

void TimeDelayEstimator::preprocessData(rclcpp::Node * node)
{
  try {
    is_valid_data_ = data_processor::checkIsValidData(
      input_, response_, params_, max_current_stddev_, ignore_thresh_);
    if (is_valid_data_) {
      const int buffer = 2;
      has_enough_input_ = data_processor::processInputData(input_, params_, buffer);
      has_enough_response_ = data_processor::processResponseData(
        node, response_, response_dot_, response_2dot_, params_, buffer);
    }
  } catch (std::runtime_error & e) {  // Handle runtime errors
    std::cerr << "[time_delay_estimator] at preprocessData runtime_error: " << e.what() <<
      std::endl;
  } catch (std::logic_error & e) {
    std::cerr << "[time_delay_estimator] at preprocessData logic_error: " << e.what() << std::endl;
  } catch (...) {  // Handle all unexpected exceptions
    RCLCPP_ERROR(node->get_logger(), "[time_delay_estimator] at preprocessData unkown error");
  }
}

void TimeDelayEstimator::processDebugData(rclcpp::Node * node)
{
  auto & de = debugger_->debug_values_;
  if (input_.raw.size() <= 5 && response_.raw.size() <= 5) {
    auto & clk = *node->get_clock();
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk,
      5000, "[time_delay_estimator] current input data size: " << input_.processed.size());
  } else {
    // debug I/O
    de.data[debugger_->DBGVAL::INPUT_RAW] = input_.raw.back();
    de.data[debugger_->DBGVAL::INPUT_FILTERED] = input_.filtered.back();
    de.data[debugger_->DBGVAL::INPUT_PROCESSED] = input_.processed.back();
    de.data[debugger_->DBGVAL::RESPONSE_RAW] = response_.raw.back();
    de.data[debugger_->DBGVAL::RESPONSE_FILTERED] = response_.filtered.back();
    de.data[debugger_->DBGVAL::RESPONSE_PROCESSED] = response_.processed.back();
    de.data[debugger_->DBGVAL::MAX_STDDEV] = static_cast<float>(max_current_stddev_ * 10.0);
    de.data[debugger_->DBGVAL::IS_VALID_DATA] = static_cast<int>(is_valid_data_);
    de.data[debugger_->DBGVAL::MIN_IGNORE_THRESH] = ignore_thresh_ * 10.0;
    // cross correlation
    de.data[debugger_->CROSS_CORRELATION::CC_DELAY] = cc_estimator_.time_delay;
    de.data[debugger_->CROSS_CORRELATION::CC_MAE_AT_TIME] = cc_estimator_.mae;
    de.data[debugger_->CROSS_CORRELATION::CC_MAE_MEAN] = cc_estimator_.stat_error.mean;
    de.data[debugger_->CROSS_CORRELATION::CC_MAE_STDDEV] = cc_estimator_.stat_error.stddev;
    de.data[debugger_->CROSS_CORRELATION::CC_DELAY_STDDEV] = cc_estimator_.stat_delay.stddev;
    de.data[debugger_->CROSS_CORRELATION::CC_DETECTION_RESULT] =
      0.5 * static_cast<float>(detection_result_);
    // primary order
    de.data[debugger_->LEAST_SQUARED::LS_DELAY] = ls_estimator_.time_delay;
    de.data[debugger_->LEAST_SQUARED::LS_MAE_AT_TIME] = ls_estimator_.mae;
    // secondary order
    de.data[debugger_->LEAST_SQUARED_SECOND::LS2_DELAY] = ls2_estimator_.time_delay;
    de.data[debugger_->LEAST_SQUARED_SECOND::LS2_MAE_AT_TIME] = ls2_estimator_.mae;
    debugger_->publishDebugValue();
    std_msgs::msg::Float32MultiArray input_data;
    std_msgs::msg::Float32MultiArray response_data;
    std_msgs::msg::Float32MultiArray estimated_data;
    auto & input_dim = input_data.layout.dim;
    input_dim.resize(1);
    input_dim[0].size = params_.total_data_size;
    auto & response_dim = response_data.layout.dim;
    response_dim.resize(1);
    response_dim[0].size = params_.total_data_size;
    auto & estimated_dim = estimated_data.layout.dim;
    estimated_dim.resize(1);
    estimated_dim[0].size = params_.total_data_size;
    int size = static_cast<int>(input_.processed.size() * (1.0 + params_.valid_delay_index_ratio));
    if (params_.is_showing_debug_info) {
      // for input & response
      for (int i = 0; i < size; i += 1) {
        if (i < static_cast<int>(input_.processed.size())) {
          input_data.data.emplace_back(input_.processed[i]);
          response_data.data.emplace_back(response_.processed[i]);
        } else {
          input_data.data.emplace_back(0);
          response_data.data.emplace_back(0);
        }
        int slide_index = i + cc_estimator_.estimated_delay_index;
        if (slide_index < static_cast<int>(response_.processed.size())) {
          estimated_data.data.emplace_back(response_.processed[slide_index]);
        } else {
          estimated_data.data.emplace_back(0);
        }
      }
      // for cross correlation
      std_msgs::msg::Float64MultiArray corr_data;
      if (cc_estimator_.cross_correlation.size() > 0) {
        for (int i = 0; i < size; i += 1) {
          if (i == cc_estimator_.estimated_delay_index) {
            corr_data.data.emplace_back(1);
          } else {
            corr_data.data.emplace_back(0);
          }
        }
        // cross correlation
        debugger_->pub_debug_input_->publish(input_data);
        debugger_->pub_debug_response_->publish(response_data);
        debugger_->pub_debug_estimated_->publish(estimated_data);
        debugger_->pub_debug_corr_->publish(corr_data);
      }
    }
  }
}

void TimeDelayEstimator::postprocessOutput(Estimator & estimator)
{
  // stash initial result
  const int ignore_count = 50;
  estimator.time_delay = math_utils::lowpassFilter(
    estimator.time_delay, estimator.time_delay_prev, params_.cutoff_hz_output,
    params_.estimation_delta_time);
  if (loop_count_ < ignore_count) {
    estimator.time_delay_prev = cc_estimator_.time_delay;
  } else {
    estimator.time_delay_prev = estimator.time_delay;
    auto & est_d = estimator.stat_delay;
    auto & est_e = estimator.stat_error;
    est_d.stddev = est_d.calcSequentialStddev(estimator.time_delay);
    est_e.stddev = est_e.calcSequentialStddev(estimator.mae);
  }
  loop_count_++;
}

void TimeDelayEstimator::setOutput(Estimator & estimator)
{
  time_delay_.time_delay = estimator.time_delay;
  time_delay_.correlation_peak = estimator.peak_correlation;
  time_delay_.time_delay_by_cross_correlation = {estimator.time_delay};
  time_delay_.first_order_model_coefficients = {estimator.w(1), estimator.w(0),
    estimator.time_delay};
  time_delay_.mean = estimator.stat_delay.mean;
  time_delay_.stddev = estimator.stat_delay.stddev;
}


tier4_calibration_msgs::msg::TimeDelay TimeDelayEstimator::estimateTimeDelay(
  rclcpp::Node * node, std::string estimator_type)
{
  try {
    const bool use_time_fitting = false;
    auto & clk = *node->get_clock();
    // use this for real time estimation only
    if (use_time_fitting) {
      math_utils::fitToTheSizeOfVector(
        input_.stamps, response_.stamps, input_.processed, response_.processed,
        params_.total_data_size, params_.num_interpolation);
    }
    // ----  Estimate Time Delay
    if (is_valid_data_ && has_enough_input_ && has_enough_response_) {
      if (estimator_type == "cc") {
        detection_result_ = estimateDelayByCrossCorrelation(
          node, input_.processed, response_.processed, cc_estimator_, name_, params_);
        postprocessOutput(cc_estimator_);
        setOutput(cc_estimator_);
      } else if (estimator_type == "ls") {
        detection_result_ = estimateDelayByLeastSquared(
          response_dot_.processed, response_.processed, input_.processed, params_);
        postprocessOutput(ls_estimator_);
        setOutput(ls_estimator_);
      } else if (estimator_type == "ls2") {
        detection_result_ = estimateDelayByLeastSquared(
          response_2dot_.processed, response_dot_.processed, response_.processed, input_.processed,
          params_);
        postprocessOutput(ls2_estimator_);
        setOutput(ls2_estimator_);
      } else {
        RCLCPP_ERROR(node->get_logger(), "[time_delay_estimator] the estimator_type is invalid");
      }
      processDebugData(node);
    } else {
      detection_result_ = DetectionResult::SLEEP;
      RCLCPP_DEBUG_STREAM_THROTTLE(
        rclcpp::get_logger("time_delay_estimator"), clk,
        2000, "[time_delay_estimator] less than data size: " << input_.processed.size());
    }
    // for publish statistics
    time_delay_.is_valid_data = is_valid_data_;
    RCLCPP_DEBUG_STREAM_THROTTLE(
      rclcpp::get_logger("time_delay_estimator"), clk,
      5000, name_ << ": (" << time_delay_.time_delay << "," << time_delay_.correlation_peak <<
        "," << time_delay_.mean << "," << time_delay_.stddev << ")");
  } catch (std::runtime_error & e) {  // Handle runtime errors
    std::cerr << "[time_delay_estimator] at estimate runtime_error: " << e.what() << std::endl;
  } catch (std::logic_error & e) {
    std::cerr << "[time_delay_estimator] at estimate logic_error: " << e.what() << std::endl;
  } catch (...) {  // Handle all unexpected exceptions
    RCLCPP_ERROR(node->get_logger(), "[time_delay_estimator] at estimate unkown error");
  }
  return time_delay_;
}
