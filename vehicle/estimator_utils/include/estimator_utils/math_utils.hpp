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


#ifndef ESTIMATOR_UTILS__MATH_UTILS_HPP_
#define ESTIMATOR_UTILS__MATH_UTILS_HPP_

#include <algorithm>
#include <cmath>
#include <deque>
#include <numeric>
#include <utility>
#include <vector>

namespace math_utils
{
inline double saturation(const double val, const double min, const double max)
{
  return std::min(std::max(val, min), max);
}
static double interpolate(const double val, const double ref, const double w)
{
  return val * (1 - w) + w * ref;
}

inline double normalize(const double val, const double min, const double max)
{
  return (val - min) / (max - min);
}

inline std::vector<double> getLinearInterpolation(const std::vector<double> & arr, int num_interp)
{
  std::vector<double> interp;
  for (size_t i = 1; i < arr.size(); i++) {
    double a = arr[i - 1];
    double b = arr[i];
    for (int j = 0; j < num_interp; j++) {
      double weight = j / static_cast<double>(num_interp);
      double val = interpolate(a, b, weight);
      interp.emplace_back(val);
    }
  }
  interp.emplace_back(arr.back());
  return interp;
}

template<class T>
int getMaximumIndexFromVector(const T & x)
{
  const auto iter = std::max_element(x.begin(), x.end());
  return std::distance(x.begin(), iter);
}
template<class T>
double getAverageFromVector(const T & arr)
{
  if (arr.size() == 0) {
    return 0;
  }
  return std::accumulate((arr).begin(), (arr).end(), 0.0) / static_cast<double>(arr.size());
}

template<class T>
double getStddevFromVector(const T & arr)
{
  if (arr.empty()) {return 0;}
  double average = getAverageFromVector(arr);
  double diff = 0;
  for (const auto & v : arr) {
    diff += std::pow(v - average, 2);
  }
  return std::sqrt(diff / static_cast<double>(arr.size()));
}

template<class T>
double getCorrelationCoefficientFromVector(const T & x, const T & y)
{
  if (x.empty()) {return 0;}
  int sz = x.size();
  double coeff = 0;
  double x_avg = getAverageFromVector(x);
  double y_avg = getAverageFromVector(y);
  double x_stddev = getStddevFromVector(x);
  double y_stddev = getStddevFromVector(y);
  if (x_stddev < 0.0001 || y_stddev < 0.0001) {
    return 0;
  }
  for (int i = 0; i < x.size(); i++) {
    coeff += (x[i] - x_avg) * (y[i] - y_avg);
  }
  coeff /= (x_stddev * y_stddev * sz);
  return coeff;
}

template<class T>
double getAverageFromVector(const T & arr, const T & w)
{
  if (arr.empty()) {return 0;}
  double sum = 0;
  double sum_w = 0;
  for (size_t i = 0; i < arr.size(); ++i) {
    sum += w[i] * arr[i];
    sum_w += w[i];
  }
  return sum / sum_w;
}

template<class T>
double getCovarianceFromVector(const T & x, const T & y, const T & w)
{
  if (x.empty()) {return 0;}
  double cov = 0;
  double sum_w = 0;
  double avg_x = getAverageFromVector(x, w);
  double avg_y = getAverageFromVector(y, w);
  for (size_t i = 0; i < x.size(); i++) {
    cov += w[i] * (x[i] - avg_x) * (y[i] - avg_y);
    sum_w += w[i];
  }
  return cov / sum_w;
}
template<class T>
double getStddevFromVector(const T & arr, const T & w)
{
  if (arr.empty()) {return 0;}
  double average = getAverageFromVector(arr, w);
  double var = 0;
  double w_sum = 0;
  for (size_t i = 0; i < arr.size(); i++) {
    var += w[i] * std::pow((arr[i] - average), 2);
    w_sum += w[i];
  }
  return std::sqrt(var / w_sum);
}

template<class T>
double getCorrelationCoefficientFromVector(const T & x, const T & y, const T & w)
{
  if (x.empty()) {return 0;}
  double x_stddev = getStddevFromVector(x, w);
  double y_stddev = getStddevFromVector(y, w);
  double xy_cov = getCovarianceFromVector(x, y, w);
  return xy_cov / (x_stddev * y_stddev);
}

inline double lowpassFilter(
  const double current_value, const double prev_value, double cutoff, const double dt)
{
  const double tau = 1 / (2 * M_PI * cutoff);
  const double a = tau / (dt + tau);
  return prev_value * a + (1 - a) * current_value;
}

/**
 * @param : arr vector like container
 * @return : avg_arr processed arr vector
 */
template<class T>
std::vector<double> getAveragedVector(const T & arr)
{
  if (arr.empty()) {return arr;}
  std::vector<double> avg_arr = {arr.begin(), arr.end()};
  double avg = std::accumulate((arr).begin(), (arr).end(), 0.0) / static_cast<double>(arr.size());
  for (auto & v : avg_arr) {
    v -= avg;
  }
  return avg_arr;
}

/**
 * @param : arr vector like container
 * @return : avg_arr processed arr vector
 */
template<class T>
std::vector<double> arrToVector(const T & arr)
{
  std::vector<double> vec = {arr.begin(), arr.end()};
  return vec;
}

/**
 * @param : arr vector like container
 * @param : slide move vector placement 0 or 1
 * @return : avg_arr processed arr vector
 */
template<class T>
std::vector<double> fitToTheSizeOfVector(const T & arr, const int size, const int num_slide = 0)
{
  if (static_cast<int>(arr.size()) < (size + num_slide)) {return {arr.begin(), arr.end()};}
  return {arr.end() - size - num_slide, arr.end() - num_slide};
}

/**
 * @param : input_stamp : deque type input stamp
 * @param : input_stamp : deque type input stamp
 * @param : input : deque type input
 * @param : response : deque type response
 * @param : size : size of final output
 */
template<class T>
void fitToTheSizeOfVector(
  const T & input_stamp, const T & response_stamp, std::vector<double> & input,
  std::vector<double> & response, const int size, const int num_slide)
{
  const size_t max_slide = num_slide;
  if (input.size() < (size + max_slide) && response.size() < (size + max_slide)) {return;}
  double in0 = *(input_stamp.end() - 1);
  double in1 = *(input_stamp.end() - 2);
  double re0 = *(response_stamp.end() - 1);
  double re1 = *(response_stamp.end() - 2);
  double d00 = std::abs(in0 - re0);
  double d10 = std::abs(in1 - re0);
  double d01 = std::abs(in0 - re1);
  double input_slide = 0;
  double response_slide = 0;
  if (d01 < d00 && d01 < d10) {
    response_slide = num_slide;
  } else if (d10 < d00 && d10 < d01) {
    input_slide = num_slide;
  }
  input = fitToTheSizeOfVector(input, size, input_slide);
  response = fitToTheSizeOfVector(response, size, response_slide);
}
/**
 *
 * @param input : input signal
 * @param response : output signal
 * @param valid_delay_index_ratio : number of shift to compare rate
 * @return corr : size of (input+num_shift) , type T correlation value
 */
template<class T>
T calcCrossCorrelationCoefficient(
  const T & input, const T & response, const double valid_delay_index_ratio)
{
  T r_input = input;
  T r_response = response;
  std::reverse(r_input.begin(), r_input.end());
  std::reverse(r_response.begin(), r_response.end());
  int T_interval = static_cast<int>(input.size() * valid_delay_index_ratio);
  T CorrCoeff(T_interval + 1, 0.0);
  /**
   * Correlation Coefficient Method
   * CorrCoeff = Cov(x1x2)/Stddev(x1)*Stddev(x2)
   */
  for (int tau = 0; tau < T_interval; tau++) {
    T cmp_input(input.size());
    T cmp_response(response.size());
    copy(r_input.begin() + tau, r_input.end(), cmp_input.begin());
    copy(r_response.begin(), r_response.end() - tau, cmp_response.begin());
    CorrCoeff.at(tau) = getCorrelationCoefficientFromVector(cmp_input, cmp_response);
  }
  return CorrCoeff;
}

/**
 *
 * @param input : input signal
 * @param response : output signal
 * @param weight : weight for correlation
 * @param valid_delay_index_ratio : number of shift to compare rate
 * @return corr : size of (input+num_shift) , type T correlation value
 */
template<class T>
T calcCrossCorrelationCoefficient(
  const T & input, const T & response, const T & weight, const double valid_delay_index_ratio)
{
  T r_input = input;
  T r_response = response;
  std::reverse(r_input.begin(), r_input.end());
  std::reverse(r_response.begin(), r_response.end());
  int T_interval = static_cast<int>(input.size() * valid_delay_index_ratio);
  T CorrCoeff(T_interval, 0.0);
  /**
   * Correlation Coefficient Method
   * CorrCoeff = Cov(x1x2)/Stddev(x1)*Stddev(x2)
   */
  for (int tau = 0; tau < T_interval - 1; tau++) {
    T cmp_input(input.size());
    T cmp_response(response.size());
    copy(r_input.begin() + tau, r_input.end(), cmp_input.begin());
    copy(r_response.begin(), r_response.end() - tau, cmp_response.begin());
    CorrCoeff.at(tau) = getCorrelationCoefficientFromVector(cmp_input, cmp_response, weight);
  }
  return CorrCoeff;
}

template<class T>
double calcMAE(const T & input, const T & response, const int delay_index)
{
  size_t sz = input.size() / 2;
  T r_input = input;
  T r_response = response;
  std::reverse(r_input.begin(), r_input.end());
  std::reverse(r_response.begin(), r_response.end());
  std::vector<double> errors;  // input - response
  for (size_t i = 0; i < sz; i++) {
    errors.emplace_back(r_input[i + delay_index] - r_response[i]);
  }
  double abs_sum = 0;
  for (size_t i = 0; i < sz; i++) {
    abs_sum += std::abs(errors[i]);
  }
  double mae = abs_sum / static_cast<double>(sz);
  return mae;
}

struct Statistics
{
  Statistics() {}
  explicit Statistics(int dim)
  {
    value = std::vector<double>(dim, 0.0);
    mean = std::vector<double>(dim, 0.0);
    variance = std::vector<double>(dim, 0.0);
    stddev = std::vector<double>(dim, 0.0);
  }
  std::vector<double> value;
  std::vector<double> mean;
  std::vector<double> variance;
  std::vector<double> stddev;
  int count = 0;
};

inline void calcSequentialStddev(Statistics & stat)
{
  auto & cnt = stat.count;
  double seq = static_cast<double>(cnt);
  for (size_t i = 0; i < stat.value.size(); i++) {
    auto & mean = stat.mean[i];
    auto & variance = stat.variance[i];
    auto & val = stat.value[i];
    auto & stddev = stat.stddev[i];
    const auto old_avg = stat.mean[i];
    mean = (seq * mean + val) / (seq + 1.0);
    variance = (seq * (variance + std::pow(old_avg, 2)) + std::pow(val, 2)) / (seq + 1.0) -
      std::pow(mean, 2);
    stddev = std::sqrt(variance);
  }
  cnt++;
}

struct Statistic
{
  int cnt = 0;
  double mean = 0;
  double variance = 0;
  double stddev = 0;
  // mean absolute error
  double mae = 0;
  // O(1) speed stddev & mean
  double calcSequentialStddev(const double val)
  {
    double old_avg = mean;
    double seq = static_cast<double>(cnt);
    mean = (seq * mean + val) / (seq + 1.0);
    variance = (seq * (variance + std::pow(old_avg, 2)) + std::pow(val, 2)) / (seq + 1.0) -
      std::pow(mean, 2);
    cnt++;
    return std::sqrt(variance);
  }
};
}  // namespace math_utils

#endif  // ESTIMATOR_UTILS__MATH_UTILS_HPP_
