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

#ifndef EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CERES_FUNCTIONS_HPP_
#define EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CERES_FUNCTIONS_HPP_

#include <ceres/ceres.h>

namespace extrinsic_tag_based_base_calibrator
{

/*
 * These methods were copied from ceres/rotation.h.
 * The reason behind the copy was the need for an implementation where the point or pose
 * was fixed and thus could not find a signature with both ceres::Jet and double arguments
 */
template <typename T1, typename T2, typename T3>
inline void UnitQuaternionRotatePoint2(const T1 q[4], const T2 pt[3], T3 result[3])
{
  const T1 t2 = q[0] * q[1];
  const T1 t3 = q[0] * q[2];
  const T1 t4 = q[0] * q[3];
  const T1 t5 = -q[1] * q[1];
  const T1 t6 = q[1] * q[2];
  const T1 t7 = q[1] * q[3];
  const T1 t8 = -q[2] * q[2];
  const T1 t9 = q[2] * q[3];
  const T1 t1 = -q[3] * q[3];
  result[0] =
    T3(2) * ((t8 + t1) * pt[0] + (t6 - t4) * pt[1] + (t3 + t7) * pt[2]) + pt[0];  // NOLINT
  result[1] =
    T3(2) * ((t4 + t6) * pt[0] + (t5 + t1) * pt[1] + (t9 - t2) * pt[2]) + pt[1];  // NOLINT
  result[2] =
    T3(2) * ((t7 - t3) * pt[0] + (t2 + t9) * pt[1] + (t5 + t8) * pt[2]) + pt[2];  // NOLINT
}

template <typename T1, typename T2, typename T3>
inline void QuaternionRotatePoint2(const T1 q[4], const T2 pt[3], T3 result[3])
{
  // 'scale' is 1 / norm(q).
  const T1 scale = T1(1) / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  // Make unit-norm version of q.
  const T1 unit[4] = {
    scale * q[0],
    scale * q[1],
    scale * q[2],
    scale * q[3],
  };

  UnitQuaternionRotatePoint2<T1, T2, T3>(unit, pt, result);
}

template <typename T1, typename T2, typename T3>
inline void UnitQuaternionRotatePoint2(
  const Eigen::Matrix<T1, 4, 1> & q, const Eigen::Matrix<T2, 3, 1> & pt,
  Eigen::Matrix<T3, 3, 1> & result)
{
  const T1 t2 = q(0) * q(1);
  const T1 t3 = q(0) * q(2);
  const T1 t4 = q(0) * q(3);
  const T1 t5 = -q(1) * q(1);
  const T1 t6 = q(1) * q(2);
  const T1 t7 = q(1) * q(3);
  const T1 t8 = -q(2) * q(2);
  const T1 t9 = q(2) * q(3);
  const T1 t1 = -q(3) * q(3);
  result(0) =
    T3(2) * ((t8 + t1) * pt(0) + (t6 - t4) * pt(1) + (t3 + t7) * pt(2)) + pt(0);  // NOLINT
  result(1) =
    T3(2) * ((t4 + t6) * pt(0) + (t5 + t1) * pt(1) + (t9 - t2) * pt(2)) + pt(1);  // NOLINT
  result(2) =
    T3(2) * ((t7 - t3) * pt(0) + (t2 + t9) * pt(1) + (t5 + t8) * pt(2)) + pt(2);  // NOLINT
}

template <typename T1, typename T2, typename T3>
inline void QuaternionRotatePoint2(
  const Eigen::Map<const Eigen::Matrix<T1, 4, 1>> & q, const Eigen::Matrix<T2, 3, 1> & pt,
  Eigen::Matrix<T3, 3, 1> & result)
{
  // 'scale' is 1 / norm(q).
  const T1 scale = T1(1) / sqrt(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3));

  // Make unit-norm version of q.

  const Eigen::Matrix<T1, 4, 1> unit = {
    scale * q(0),
    scale * q(1),
    scale * q(2),
    scale * q(3),
  };

  UnitQuaternionRotatePoint2<T1, T2, T3>(unit, pt, result);
}

template <typename T1, typename T2, typename T3>
inline void QuaternionRotatePoint2(
  const Eigen::Matrix<T1, 4, 1> & q, const Eigen::Matrix<T2, 3, 1> & pt,
  Eigen::Matrix<T3, 3, 1> & result)
{
  // 'scale' is 1 / norm(q).
  const T1 scale = T1(1) / sqrt(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3));

  // Make unit-norm version of q.

  const Eigen::Matrix<T1, 4, 1> unit = {
    scale * q(0),
    scale * q(1),
    scale * q(2),
    scale * q(3),
  };

  UnitQuaternionRotatePoint2<T1, T2, T3>(unit, pt, result);
}

template <typename T>
Eigen::Matrix<T, 3, 3> rotationMatrixFromYaw(T yaw)
{
  const T cos = ceres::cos(yaw);
  const T sin = ceres::sin(yaw);
  Eigen::Matrix<T, 3, 3> rotation;
  rotation << cos, -sin, T(0.0), sin, cos, T(0.0), T(0.0), T(0.0), T(1.0);
  return rotation;
}

template <typename T1, typename T2>
void transformPoint(
  const Eigen::Map<const Eigen::Matrix<T1, 4, 1>> & rotation,
  const Eigen::Map<const Eigen::Matrix<T1, 3, 1>> & translation,
  const Eigen::Matrix<T2, 3, 1> & input_point, Eigen::Matrix<T2, 3, 1> & transformed_point)
{
  QuaternionRotatePoint2(rotation, input_point, transformed_point);
  transformed_point.x() += translation.x();
  transformed_point.y() += translation.y();
  transformed_point.z() += translation.z();
}

template <typename T1, typename T2>
void transformPoint(
  const Eigen::Map<const Eigen::Matrix<T1, 4, 1>> & rotation_3d, const T1 & z,
  const Eigen::Matrix<T1, 3, 3> & rotation_2d,
  const Eigen::Map<const Eigen::Matrix<T1, 2, 1>> & translation_2d,
  const Eigen::Matrix<T2, 3, 1> & input_point, Eigen::Matrix<T2, 3, 1> & transformed_point)
{
  Eigen::Matrix<T2, 3, 1> aux = rotation_2d * input_point;
  aux.x() += translation_2d.x();
  aux.y() += translation_2d.y();

  // Compute the inverse of the projected ground pose
  const Eigen::Matrix<T1, 4, 1> rotation_3d_inv(
    rotation_3d(0), -rotation_3d(1), -rotation_3d(2), -rotation_3d(3));
  const Eigen::Matrix<T1, 3, 1> translation_3d(T1(0), T1(0), z);
  Eigen::Matrix<T1, 3, 1> translation_3d_inv;
  QuaternionRotatePoint2(rotation_3d_inv, translation_3d, translation_3d_inv);

  QuaternionRotatePoint2(rotation_3d_inv, aux, transformed_point);
  transformed_point -= translation_3d_inv;
}

template <typename T1, typename T2>
void transformCorners(
  const Eigen::Map<const Eigen::Matrix<T1, 4, 1>> & rotation,
  const Eigen::Map<const Eigen::Matrix<T1, 3, 1>> & translation,
  const Eigen::Matrix<T2, 3, 1> input_corners[4], Eigen::Matrix<T2, 3, 1> transformed_corners[4])
{
  for (int i = 0; i < 4; i++) {
    transformPoint(rotation, translation, input_corners[i], transformed_corners[i]);
  }
}

template <typename T1, typename T2>
void transformCorners2(
  const Eigen::Map<const Eigen::Matrix<T1, 4, 1>> & rotation_3d, const T1 & z,
  const Eigen::Matrix<T1, 3, 3> & rotation_2d,
  const Eigen::Map<const Eigen::Matrix<T1, 2, 1>> & translation_2d,
  const Eigen::Matrix<T2, 3, 1> input_corners[4], Eigen::Matrix<T2, 3, 1> transformed_corners[4])
{
  for (int i = 0; i < 4; i++) {
    transformPoint(
      rotation_3d, z, rotation_2d, translation_2d, input_corners[i], transformed_corners[i]);
  }
}

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CERES_FUNCTIONS_HPP_
