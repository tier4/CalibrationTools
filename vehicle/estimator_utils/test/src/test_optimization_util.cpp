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

#include <gtest/gtest.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>

#include <cassert>
#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>
#include <random>
#include <utility>
#include <vector>

#include "estimator_utils/optimization_utils.hpp"

TEST(optimization_utils, change_abs_min)
{
  using optimization_utils::change_abs_min;
  int dim_t = 3;
  int diff_t = 3;
  Eigen::VectorXd w = Eigen::MatrixXd::Zero(diff_t, 1);
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(diff_t, dim_t);
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(diff_t, 1);
  // 13^2 14^2 15^2
  // 4*13^2 4*14^2 4*15^2
  // Y = 4*X^2 -> w = [0,0,4]
  X << 2, 24, 169, 2, 26, 196, 2, 28, 225;
  Y << 676, 784, 900;
  optimization_utils::getSolutionByLeastSquared(X, Y, w);
  EXPECT_DOUBLE_EQ(w(0), 0.0);
  EXPECT_DOUBLE_EQ(w(1), 0.0);
  EXPECT_DOUBLE_EQ(w(2), 4.0);
}

TEST(optimization_utils, getSolutionByLeastSquared)
{
  using optimization_utils::getSolutionByLeastSquared;
  int dim_t = 3;
  int diff_t = 3;
  Eigen::VectorXd w = Eigen::MatrixXd::Zero(diff_t, 1);
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(diff_t, dim_t);
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(diff_t, 1);
  // 13^2 14^2 15^2
  // 4*13^2 4*14^2 4*15^2
  // Y = 4*X^2 -> w = [0,0,4]
  X << 2, 24, 169, 2, 26, 196, 2, 28, 225;
  Y << 676, 784, 900;
  getSolutionByLeastSquared(X, Y, w);
  EXPECT_DOUBLE_EQ(w(0), 0.0);
  EXPECT_DOUBLE_EQ(w(1), 0.0);
  EXPECT_DOUBLE_EQ(w(2), 4.0);
}

TEST(optimization_utils, getErrorNorm)
{
  using optimization_utils::getErrorNorm;
  int dim_t = 3;
  int diff_t = 3;
  Eigen::VectorXd w = Eigen::MatrixXd::Zero(diff_t, 1);
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(diff_t, dim_t);
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(diff_t, 1);
  // 13^2 14^2 15^2
  // 4*13^2 4*14^2 4*15^2
  // Y = 4*X^2 -> w = [0,0,4]
  X << 2, 24, 169, 2, 26, 196, 2, 28, 225;
  Y << 676, 784, 900;
  w << 0, 0, 4;
  double error = getErrorNorm(X, Y, w);
  EXPECT_DOUBLE_EQ(error, 0.0);
}

TEST(optimization_utils, getSecondaryCentralDifference)
{
  using optimization_utils::getSecondaryCentralDifference;
  double x0 = 0;
  double x1 = 2;
  double x2 = 4;
  double dt = 1;
  double xd = getSecondaryCentralDifference(x2, x0, dt);
  double xdd = getSecondaryCentralDifference(x2, x1, x0, dt);

  EXPECT_DOUBLE_EQ(xd, 2);
  EXPECT_DOUBLE_EQ(xdd, 0);
}
