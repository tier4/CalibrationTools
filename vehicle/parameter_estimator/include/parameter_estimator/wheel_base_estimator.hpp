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

#ifndef PARAMETER_ESTIMATOR__WHEEL_BASE_ESTIMATOR_HPP_
#define PARAMETER_ESTIMATOR__WHEEL_BASE_ESTIMATOR_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "estimator_utils/estimator_base.hpp"
#include "estimator_utils/math_utils.hpp"
#include "estimator_utils/optimization_utils.hpp"
#include "parameter_estimator/debugger.hpp"
#include "parameter_estimator/parameters.hpp"

class WheelBaseEstimator : public EstimatorBase
{
public:
  explicit WheelBaseEstimator(
    rclcpp::Node * node, const Params & p, double cov, const double ff, const double est);
  ~WheelBaseEstimator() {}
  void setData(const VehicleData & v) {data_ = v;}

private:
  VehicleData data_;
  Params params_;
  std::unique_ptr<Debugger> debugger_;
  double covariance_;
  double forgetting_factor_;
  double estimated_;
  double error_;
  bool estimate();
  bool checkIsValidData();
  void preprocessData() {}
  void postprocessOutput();
  void publishData();
};

#endif  // PARAMETER_ESTIMATOR__WHEEL_BASE_ESTIMATOR_HPP_
