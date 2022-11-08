// Copyright 2022 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_

#include <Eigen/Core>
#include <Eigen/SVD>

#include <iostream>
#include <limits>
#include <vector>

namespace extrinsic_tag_based_base_calibrator
{

/*
 * Method to find the average of a set of rotation quaternions using Singular Value Decomposition
 * Snipped taken from https://gist.github.com/PeteBlackerThe3rd/f73e9d569e29f23e8bd828d7886636a0
 * The algorithm used is described here:
 * https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070017872.pdf
 */
Eigen::Vector4f quaternionAverage(std::vector<Eigen::Vector4f> quaternions)
{
  if (quaternions.size() == 0) {
    std::cout << "Error trying to calculate the average quaternion of an empty set!\n";
    return Eigen::Vector4f::Zero();
  }

  // first build a 4x4 matrix which is the elementwise sum of the product of each quaternion with
  // itself
  Eigen::Matrix4f A = Eigen::Matrix4f::Zero();

  for (std::size_t q = 0; q < quaternions.size(); ++q) {
    A += quaternions[q] * quaternions[q].transpose();
  }

  // normalise with the number of quaternions
  A /= quaternions.size();

  // Compute the SVD of this 4x4 matrix
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::VectorXf singular_values = svd.singularValues();
  Eigen::MatrixXf U = svd.matrixU();

  // find the eigen vector corresponding to the largest eigen value
  int largest_eigen_value_index = -1;
  float largest_eigen_value = -std::numeric_limits<float>::max();

  for (int i = 0; i < singular_values.rows(); ++i) {
    if (singular_values(i) > largest_eigen_value) {
      largest_eigen_value = singular_values(i);
      largest_eigen_value_index = i;
    }
  }

  Eigen::Vector4f average;
  average(0) = U(0, largest_eigen_value_index);
  average(1) = U(1, largest_eigen_value_index);
  average(2) = U(2, largest_eigen_value_index);
  average(3) = U(3, largest_eigen_value_index);

  return average;
}

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_
