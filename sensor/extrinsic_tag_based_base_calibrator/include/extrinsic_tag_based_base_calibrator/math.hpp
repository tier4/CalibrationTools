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
#include <extrinsic_tag_based_base_calibrator/types.hpp>

#include <iostream>
#include <limits>
#include <memory>
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

std::array<cv::Vec3f, 4> tagPoseToCorners(const cv::Affine3f & pose, float size)
{
  std::array<cv::Vec3f, 4> templates{
    cv::Vec3f{-1.0, 1.0, 0.0}, cv::Vec3f{1.0, 1.0, 0.0}, cv::Vec3f{1.0, -1.0, 0.0},
    cv::Vec3f{-1.0, -1.0, 0.0}};

  return std::array<cv::Vec3f, 4>{
    pose * (0.5f * size * templates[0]), pose * (0.5f * size * templates[1]),
    pose * (0.5f * size * templates[2]), pose * (0.5f * size * templates[3])};
}

bool computeGroundPlane(const std::vector<cv::Vec3f> & points, cv::Affine3f & ground_pose)
{
  int num_points = static_cast<int>(points.size());

  if (num_points == 0) {
    return false;
  }

  cv::Mat_<float> pca_input = cv::Mat_<float>(num_points, 3);

  for (int i = 0; i < num_points; i++) {
    pca_input(i, 0) = points[i](0);
    pca_input(i, 1) = points[i](1);
    pca_input(i, 2) = points[i](2);
  }

  cv::PCA pca_analysis(pca_input, cv::Mat_<float>(), cv::PCA::DATA_AS_ROW);

  cv::Matx33f rotation = cv::Matx33f(pca_analysis.eigenvectors).inv();
  cv::Vec3f translation = cv::Vec3f(pca_analysis.mean);

  ground_pose = cv::Affine3f(rotation, translation);

  return true;
}

bool computeGroundPlane(
  const std::vector<std::shared_ptr<cv::Affine3f>> & poses, float tag_size,
  cv::Affine3f & ground_pose)
{
  std::vector<cv::Vec3f> points;

  for (const auto & pose : poses) {
    std::array<cv::Vec3f, 4> corners = tagPoseToCorners(*pose, tag_size);
    points.insert(points.end(), corners.begin(), corners.end());
  }

  return computeGroundPlane(points, ground_pose);
}

cv::Affine3f computeBaseLink(
  const cv::Affine3f & left_wheel_pose, const cv::Affine3f right_wheel_pose,
  const cv::Affine3f & ground_pose)
{
  // Compute the base link center
  cv::Vec3f tag_center = 0.5 * (left_wheel_pose.translation() + right_wheel_pose.translation());
  cv::Vec3f base_link_translation = ground_pose.inv() * tag_center;
  base_link_translation(2) = 0.f;
  base_link_translation = ground_pose * base_link_translation;

  cv::Vec3f base_link_direction = base_link_translation / cv::norm(base_link_translation);

  cv::Vec3f base_link_z_axis(ground_pose.rotation().col(2).val);
  float factor = base_link_z_axis.dot(base_link_direction) > 0.f ? -1.f : 1.f;
  base_link_z_axis = factor * base_link_z_axis;

  cv::Vec3f base_link_y_axis = left_wheel_pose.translation() - right_wheel_pose.translation();
  base_link_y_axis = ground_pose.rotation().inv() * base_link_y_axis;
  base_link_y_axis(2) = 0.f;
  base_link_y_axis = base_link_y_axis / cv::norm(base_link_y_axis);
  base_link_y_axis = ground_pose.rotation() * base_link_y_axis;

  cv::Vec3f base_link_x_axis = base_link_y_axis.cross(base_link_z_axis);

  auto fill_rotation_from_column = [](cv::Matx33f & rotation, const cv::Vec3f column, int index) {
    rotation(0, index) = column(0);
    rotation(1, index) = column(1);
    rotation(2, index) = column(2);
  };

  cv::Matx33f base_link_rotation;
  fill_rotation_from_column(base_link_rotation, base_link_x_axis, 0);
  fill_rotation_from_column(base_link_rotation, base_link_y_axis, 1);
  fill_rotation_from_column(base_link_rotation, base_link_z_axis, 2);

  return cv::Affine3f(base_link_rotation, base_link_translation);
}

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__MATH_HPP_