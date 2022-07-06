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

#ifndef TIME_DELAY_ESTIMATOR__TIME_DELAY_ESTIMATOR_HPP_
#define TIME_DELAY_ESTIMATOR__TIME_DELAY_ESTIMATOR_HPP_

#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>
#include <utility>
#include <vector>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "tier4_calibration_msgs/msg/time_delay.hpp"
#include "estimator_utils/math_utils.hpp"
#include "estimator_utils/optimization_utils.hpp"
#include "time_delay_estimator/data_processor.hpp"
#include "time_delay_estimator/debugger.hpp"
#include "time_delay_estimator/parameters.hpp"

class TimeDelayEstimator
{
private:
  enum class DetectionResult : std::int8_t
  {
    BELOW_THRESH = 0,
    SLEEP = 1,
    DETECTED = 2,
  };
  bool has_enough_input_ = false;
  bool has_enough_response_ = false;
  tier4_calibration_msgs::msg::TimeDelay time_delay_;
  int loop_count_ = 0;
  Estimator cc_estimator_;
  Estimator ls_estimator_;
  Estimator ls2_estimator_;
  std::vector<double> weights_for_data_;
  std::string name_;
  bool is_valid_data_ = false;
  double max_current_stddev_ = 0;
  double ignore_thresh_;
  DetectionResult detection_result_;
  data_processor::ForgetResult forget_result_;

  void postprocessOutput(Estimator & estimator);
  void setOutput(Estimator & estimator);
  bool calcStatistics(
    math_utils::Statistic & stat_delay, math_utils::Statistic & stat_error, const double delay,
    const double mae);

  /**
  * @brief : get time delay from stored input & response
  * @param input : vector like array
  * @param response : vector like array
  * @param corr : correlation value
  * @param params : parameters for setting
  * @return Correlation type value
  **/
  enum DetectionResult estimateDelayByCrossCorrelation(
    rclcpp::Node * node, const std::vector<double> & input,
    const std::vector<double> & response, Estimator & corr,
    std::string name, const Params & params);

  enum DetectionResult estimateDelayByLeastSquared(
    const std::vector<double> & x2dot, const std::vector<double> & x_dot,
    const std::vector<double> & x, const std::vector<double> & u, const Params & params);
  enum DetectionResult estimateDelayByLeastSquared(
    const std::vector<double> & x_dot, const std::vector<double> & x, const std::vector<double> & u,
    const Params & params);

public:
  Data input_;
  Data response_;
  Data response_dot_;
  Data response_2dot_;
  bool checkIsValidData();
  void estimate();
  void preprocessData(rclcpp::Node * node);
  void processDebugData(rclcpp::Node * node);
  void resetEstimator();
  Params params_;
  /**
   * @brief : initialize module
   * @param name : which delay to estimate
   * @param total_data_size : total sample data to estimate
   * @param use_weight_for_cross_correlation : weight decay
   * @return : success
   **/
  TimeDelayEstimator(
    rclcpp::Node * node, const Params & params, const std::string & name, int total_data_size,
    bool use_weight_for_cross_correlation);
  tier4_calibration_msgs::msg::TimeDelay estimateTimeDelay(
    rclcpp::Node * node,
    std::string estimator_type);
  virtual ~TimeDelayEstimator() {}

  std::unique_ptr<Debugger> debugger_;
};
#endif  //  TIME_DELAY_ESTIMATOR__TIME_DELAY_ESTIMATOR_HPP_
