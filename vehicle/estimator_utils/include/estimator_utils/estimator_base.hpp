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

#ifndef ESTIMATOR_UTILS__ESTIMATOR_BASE_HPP_
#define ESTIMATOR_UTILS__ESTIMATOR_BASE_HPP_

#include <vector>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "tier4_calibration_msgs/msg/estimation_result.hpp"
#include "estimator_utils/math_utils.hpp"

class EstimatorBase
{
public:
  EstimatorBase() {}
  virtual ~EstimatorBase() {}
  virtual bool estimate() = 0;
  virtual void preprocessData() = 0;
  virtual bool checkIsValidData() = 0;
  virtual void postprocessOutput() = 0;
  virtual void publishData() = 0;
  bool getIsValidData() {return is_valid_data_;}
  bool getIsValidEstimation() {return is_valid_estimation_;}
  using outputType = tier4_calibration_msgs::msg::EstimationResult;
  void createPublisher(const std::string & name, rclcpp::Node * node)
  {
    // QoS setup
    static constexpr std::size_t queue_size = 1;
    rclcpp::QoS durable_qos(queue_size);
    durable_qos.transient_local();  // option for latching

    pub_estimated_ =
      rclcpp::create_publisher<outputType>(node, "output/" + name, durable_qos);

    result_msgs_.result.resize(5, 0);
    result_msgs_.result_mean.resize(5, 0);
    result_msgs_.result_stddev.resize(5, 0);
    result_msgs_.absolute_error.resize(5, 0);
    result_msgs_.mean_absolute_error.resize(5, 0);
    result_msgs_.stddev_absolute_error.resize(5, 0);
  }
  void calcStatistics()
  {
    math_utils::calcSequentialStddev(result_statistics_);
    math_utils::calcSequentialStddev(error_statistics_);
  }
  void publishResult()
  {
    result_msgs_.result = result_statistics_.value;
    result_msgs_.result_mean = result_statistics_.mean;
    result_msgs_.result_stddev = result_statistics_.stddev;
    result_msgs_.absolute_error = error_statistics_.value;
    result_msgs_.mean_absolute_error = error_statistics_.mean;
    result_msgs_.stddev_absolute_error = error_statistics_.stddev;
    pub_estimated_->publish(result_msgs_);
  }
  rclcpp::Publisher<outputType>::SharedPtr pub_estimated_;
  outputType result_msgs_;
  math_utils::Statistics result_statistics_;
  math_utils::Statistics error_statistics_;
  int seq_ = 0;
  bool is_valid_data_ = false;
  bool is_valid_estimation_ = false;

  void processData() {is_valid_data_ = checkIsValidData();}
  /**
   * data flow
   * if data is valid data:
   *  1. preprocess data
   *  2. estimate parameter
   *  3. post process output
   *  publish output
   **/
  void Run()
  {
    try {
      if (is_valid_data_) {
        is_valid_estimation_ = estimate();
        if (is_valid_data_ && is_valid_estimation_) {
          postprocessOutput();
          if (seq_ > 50) {calcStatistics();}
          seq_++;
        }
      }
      publishData();
      publishResult();
    } catch (std::runtime_error & e) {  // Handle runtime errors
      std::cerr << "[parameter_estimator] runtime_error: " << e.what() << std::endl;
    } catch (std::logic_error & e) {
      std::cerr << "[parameter_estimator] logic_error: " << e.what() << std::endl;
    } catch (...) {  // Handle all unexpected exceptions
      std::cerr << "[parameter_estimator] unkown error" << std::endl;
    }
  }
};

#endif  // ESTIMATOR_UTILS__ESTIMATOR_BASE_HPP_
