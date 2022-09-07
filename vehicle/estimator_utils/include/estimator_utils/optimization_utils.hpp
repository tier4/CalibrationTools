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

#ifndef ESTIMATOR_UTILS__OPTIMIZATION_UTILS_HPP_
#define ESTIMATOR_UTILS__OPTIMIZATION_UTILS_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/LU>

#include <vector>

namespace optimization_utils
{
inline void estimateByRLS(
  double & est, double & cov, const double zn, const double ff, const double y)
{
  const double coef = (cov * zn) / (ff + zn * cov * zn);
  cov = (cov - coef * zn * cov) / ff;
  const double error = y - zn * est;
  est = est + coef * error;
}

inline void estimateByRLS(
  Eigen::MatrixXd & est, Eigen::MatrixXd & cov, const Eigen::MatrixXd & zn,
  const Eigen::MatrixXd & ff, const Eigen::MatrixXd & y)
{
  /**
   * coef_n=(cov_n-1*zn_n)/(rho_n+zn^T_n*cov_n-1*zn_n)
   * cov_n=[cov_n-1 - coef*zn^T*cov_n-1)]
   * th_n=th_n-1+coef_n*epsilon_n
   * eps_n=y_n-zn^T_n*th_n-1
   */
  Eigen::MatrixXd znT = zn.transpose();
  const Eigen::MatrixXd coef = (cov * zn) / ((ff + znT * cov * zn)(0, 0));
  cov = (cov - coef * znT * cov) / ff(0, 0);
  const Eigen::MatrixXd error = y - znT * est;
  est = est + coef * error(0, 0);
}

/**
 * @param x_t latter value
 * @param t_x previous value
 * @param dt delta time
 * @return secondary central difference
 */
inline double getSecondaryCentralDifference(const double x_t, const double t_x, const double dt)
{
  return (x_t - t_x) / (2.0 * dt);
}

inline double getSecondaryCentralDifference(
  const double x_t, const double u, const double t_x, const double dt)
{
  return (x_t - u * 2.0 + t_x) / std::pow(dt, 2);
}

static void vectorToMatrix(
  const std::vector<double> & x_dot, const std::vector<double> & x, const std::vector<double> & u,
  Eigen::MatrixXd & Y, Eigen::MatrixXd & X)
{
  int dim_t = x_dot.size();
  X = Eigen::MatrixXd::Zero(dim_t, 2);
  Y = Eigen::MatrixXd::Zero(dim_t, 1);
  for (size_t i = 0; i < x_dot.size(); ++i) {
    X(i, 0) = x_dot[i];
    X(i, 1) = x[i];
    Y(i, 0) = u[i];
  }
}

static void vectorToMatrix(
  const std::vector<double> & x2dot, const std::vector<double> & x_dot,
  const std::vector<double> & x, const std::vector<double> & u, Eigen::MatrixXd & Y,
  Eigen::MatrixXd & X)
{
  int dim_t = x_dot.size();
  X = Eigen::MatrixXd::Zero(dim_t, 3);
  Y = Eigen::MatrixXd::Zero(dim_t, 1);
  for (size_t i = 0; i < x_dot.size(); ++i) {
    X(i, 0) = x2dot[i];
    X(i, 1) = x_dot[i];
    X(i, 2) = x[i];
    Y(i, 0) = u[i];
  }
}

/**
 * @brief get solution by least squared (LS) method
 * @param X : differential matrix
 * @param Y : target matrix
 * @param w : solved by LS
 *
 * E  = || Xw - Y ||^2
 * to get minimum E -> E' = 0
 * Xt * X * w = Xt * Y
 */
static void getSolutionByLeastSquared(
  const Eigen::MatrixXd & X, const Eigen::MatrixXd & Y, Eigen::VectorXd & w)
{
  Eigen::MatrixXd XtX = X.transpose() * X;
  // print(XtX);
  Eigen::MatrixXd b = X.transpose() * Y;
  // solve Ax=b in this case A=XtX u=w(desired) b=b
  Eigen::FullPivLU<Eigen::MatrixXd> lu(XtX);
  w = lu.solve(b);
}

/**
 * @brief get error norm with optimized coefficient
 * @param X : differential matrix
 * @param Y : target matrix
 * @param w : solved by LS
 * @return : normalized error
 */
static double getErrorNorm(
  const Eigen::MatrixXd & X, const Eigen::MatrixXd & Y, const Eigen::VectorXd & w)
{
  Eigen::MatrixXd E = X * w - Y;
  double error = 0;
  for (size_t i = 0; i < static_cast<size_t>(E.rows()); i++) {
    error += std::abs(E(i, 0));
  }
  error /= static_cast<double>(E.rows());
  return error;
}

inline double getLeastSquaredError(
  const std::vector<double> & x_dot, const std::vector<double> & x, const std::vector<double> & u,
  Eigen::VectorXd & w)
{
  int dim_t = x_dot.size();
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_t, 2);
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(dim_t, 1);
  vectorToMatrix(x_dot, x, u, Y, X);
  getSolutionByLeastSquared(X, Y, w);
  return getErrorNorm(X, Y, w);
}

inline double getLeastSquaredError(
  const std::vector<double> & x2dot, const std::vector<double> & x_dot,
  const std::vector<double> & x, const std::vector<double> & u, Eigen::VectorXd & w)
{
  int dim_t = u.size();
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(dim_t, 3);
  Eigen::MatrixXd Y = Eigen::MatrixXd::Zero(dim_t, 1);
  vectorToMatrix(x2dot, x_dot, x, u, Y, X);
  getSolutionByLeastSquared(X, Y, w);
  return getErrorNorm(X, Y, w);
}

template <class T>
bool change_abs_min(T & a, const T & b)
{
  if (std::abs(b) < std::abs(a)) {
    a = std::abs(b);
    return true;
  }
  return false;
}

}  // namespace optimization_utils

#endif  // ESTIMATOR_UTILS__OPTIMIZATION_UTILS_HPP_
