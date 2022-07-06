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

#ifndef PARAMETER_ESTIMATOR__GEAR_RATIO_ESTIMATOR_HPP_
#define PARAMETER_ESTIMATOR__GEAR_RATIO_ESTIMATOR_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>
#include <memory>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "estimator_utils/estimator_base.hpp"
#include "estimator_utils/math_utils.hpp"
#include "estimator_utils/optimization_utils.hpp"
#include "parameter_estimator/debugger.hpp"
#include "parameter_estimator/parameters.hpp"

class GearRatioEstimator : public EstimatorBase
{
public:
  GearRatioEstimator(
    rclcpp::Node * node, const Params & p, const double & cov, const double ff,
    const std::vector<double> & est);
  ~GearRatioEstimator() {}
  void setData(const VehicleData & v) {data_ = v;}

private:
  VehicleData data_;
  Params params_;
  int dim_x_;
  std::unique_ptr<Debugger> debugger_;
  Eigen::MatrixXd estimated_;
  Eigen::MatrixXd covariance_;
  Eigen::MatrixXd forgetting_factor_;
  double error_;
  bool estimate();
  bool checkIsValidData();
  void preprocessData() {}
  void postprocessOutput();
  void publishData();
};

#endif  // PARAMETER_ESTIMATOR__GEAR_RATIO_ESTIMATOR_HPP_
