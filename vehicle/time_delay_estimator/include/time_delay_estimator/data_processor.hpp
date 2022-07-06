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

#ifndef TIME_DELAY_ESTIMATOR__DATA_PROCESSOR_HPP_
#define TIME_DELAY_ESTIMATOR__DATA_PROCESSOR_HPP_

#include <cmath>
#include <deque>
#include <numeric>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "estimator_utils/math_utils.hpp"
#include "estimator_utils/optimization_utils.hpp"
#include "time_delay_estimator/parameters.hpp"

struct Data
{
  Data()
  : value{0.0}, p_value{0.0} {}
  double value;
  double p_value = 0;
  double stamp = 0;
  std::deque<double> stamps = {};
  std::deque<double> validation = {};
  std::deque<double> raw = {};
  std::deque<double> filtered = {};
  std::vector<double> processed;

  void setValue(const double val, const double time)
  {
    value = val;
    stamp = time;
  }
};

struct Result
{
  int estimated_delay_index = 0;
  double time_delay = 0;
  double time_delay_stddev = 0;
  double time_delay_prev = 0;
  double time_delay_default = 0;
  bool is_valid_data = false;
};

struct Evaluation
{
  math_utils::Statistic stat_delay;
  math_utils::Statistic stat_error;
  // root squared error
  double mae = 0;
  double mae_stddev = 0;
  double mae_mean = 0;
  double mae_max = 0;
  double mae_min = 0;
};

struct Estimator : public Result, Evaluation
{
  std::vector<double> cross_correlation;
  double peak_correlation = 0;
  Eigen::VectorXd w = Eigen::MatrixXd::Zero(3, 1);
};

struct MinMax
{
  double min;
  double max;
};

namespace data_processor
{
enum class ForgetResult : std::int8_t
{
  CURRENT = 0,
  NONE = 1,
  OLD = 2,
  EXCEPTION = -1,
};
/**
  * @brief : emplace back new data and filtering
  * @param data : filtered data
  * @param data_dot :  filtered diff data
  * @param data_2dot : filtered diff2 data
  * @param params : config for preprocessor
  * @param buffer data for fitting
  * @return : has enough data to estimate
  **/
bool processResponseData(
  rclcpp::Node * node, Data & data, Data & data_dot, Data & data_2dot,
  const Params & params, const int buffer = 0);

/**
  * @brief : emplace back new data and filtering
  * @param data : raw filtered processed data
  * @param params : config for preprocessor
  * @param buffer data for fitting
  * @return : has enough data to estimate
  **/
bool processInputData(
  Data & data, const Params & params, const int buffer = 0);

/**
  * @brief : forget no feature data to avoid over fitting
  * @param input : input data
  * @param response : response data
  * @return : forget result 0: None 1:OLD 2:NEW
  **/
bool checkIsValidData(
  Data & data, Data & response, const Params & params, double & max_stddev,
  const double & ignore_thresh);

}  // namespace data_processor

#endif  //  TIME_DELAY_ESTIMATOR__DATA_PROCESSOR_HPP_
