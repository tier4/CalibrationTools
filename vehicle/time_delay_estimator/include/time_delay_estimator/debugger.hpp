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

#ifndef TIME_DELAY_ESTIMATOR__DEBUGGER_HPP_
#define TIME_DELAY_ESTIMATOR__DEBUGGER_HPP_

// ros depend
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
// ros pkg depend
#include "time_delay_estimator/parameters.hpp"

class Debugger
{
public:
  Debugger(rclcpp::Node * node, const std::string & name)
  {
    // QoS setup
    static constexpr std::size_t queue_size = 1;
    rclcpp::QoS durable_qos(queue_size);
    durable_qos.transient_local();  // option for latching

    pub_debug_ =
      node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "~/debug_values/" + name, durable_qos);
    pub_debug_input_ =
      node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "~/debug_values/" + name + "_input", durable_qos);
    pub_debug_response_ =
      node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "~/debug_values/" + name + "_response", durable_qos);
    pub_debug_estimated_ =
      node->create_publisher<std_msgs::msg::Float32MultiArray>(
      "~/debug_values/" + name + "_estimated", durable_qos);
    pub_debug_corr_ =
      node->create_publisher<std_msgs::msg::Float64MultiArray>(
      "~/debug_values/" + name + "_correlation", durable_qos);
    debug_values_.data.resize(num_debug_values_, 0.0);
  }

  /* Debug */
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_input_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_response_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_debug_estimated_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_debug_corr_;

  /**
  * @brief : for rqt_multiplot
  * @return : if publishing correlation data or not
  **/
  // debug processed data
  static constexpr std::uint8_t num_debug_values_ = 40;
  mutable std_msgs::msg::Float32MultiArray debug_values_;
  enum DBGVAL : std::uint8_t
  {
    INPUT_RAW = 0,           // [0] raw input
    INPUT_FILTERED = 1,      // [1] filtered input
    INPUT_PROCESSED = 2,     // [2] processed input
    RESPONSE_RAW = 3,        // [3] raw response
    RESPONSE_FILTERED = 4,   // [4] filtered response
    RESPONSE_PROCESSED = 5,  // [5] processed response
    MAX_STDDEV = 6,          // [6] input/response stddev
    IS_VALID_DATA = 7,       // [7] ignore current data 0 ,old data 1
    MIN_IGNORE_THRESH = 8,   // [8] min ignore threshold
  };
  enum CROSS_CORRELATION : std::uint8_t
  {
    CC_DELAY_RAW = 10,         // [10] estimate delay raw
    CC_DELAY = 11,             // [11] estimate delay processed
    CC_CORRELATION_PEAK = 12,  // [12] correlation coefficient
    CC_MAE_AT_TIME = 13,       // [13] root squared error
    CC_MAE_MEAN = 14,          // [14] accumulated mae mean
    CC_MAE_STDDEV = 15,        // [15] accumulated mae stddev
    CC_DETECTION_RESULT = 16,  // [15] detection result
    CC_DELAY_STDDEV = 17,      // [15] accumulated mae stddev
  };

  enum LEAST_SQUARED : std::uint8_t
  {
    LS_DELAY_RAW = 20,    // [20] estimate delay
    LS_DELAY = 21,        // [21] estimate delay
    LS_MAE_AT_TIME = 23,  // [23] root squared error
  };
  enum LEAST_SQUARED_SECOND : std::uint8_t
  {
    LS2_DELAY_RAW = 30,    // [30] estimate delay
    LS2_DELAY = 31,        // [31] estimate delay
    LS2_MAE_AT_TIME = 33,  // [33] root squared error
  };
  void publishDebugValue() {pub_debug_->publish(debug_values_);}
  ~Debugger() {}
};

#endif  //  TIME_DELAY_ESTIMATOR__DEBUGGER_HPP_
