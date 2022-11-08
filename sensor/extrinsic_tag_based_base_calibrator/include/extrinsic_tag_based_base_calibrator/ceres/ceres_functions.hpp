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

namespace extrinsic_tag_based_base_calibrator
{

/*
 * These methods were copied from ceres/rotation.h.
 * The reason behind the copy was the need for an implementation where the point being transformed
 * was fixed and thus could not find a signature with both ceres::Jet and double arguments
 */
template <typename T>
inline void UnitQuaternionRotatePointDouble(const T q[4], const double pt[3], T result[3])
{
  const T t2 = q[0] * q[1];
  const T t3 = q[0] * q[2];
  const T t4 = q[0] * q[3];
  const T t5 = -q[1] * q[1];
  const T t6 = q[1] * q[2];
  const T t7 = q[1] * q[3];
  const T t8 = -q[2] * q[2];
  const T t9 = q[2] * q[3];
  const T t1 = -q[3] * q[3];
  result[0] = T(2) * ((t8 + t1) * pt[0] + (t6 - t4) * pt[1] + (t3 + t7) * pt[2]) + pt[0];  // NOLINT
  result[1] = T(2) * ((t4 + t6) * pt[0] + (t5 + t1) * pt[1] + (t9 - t2) * pt[2]) + pt[1];  // NOLINT
  result[2] = T(2) * ((t7 - t3) * pt[0] + (t2 + t9) * pt[1] + (t5 + t8) * pt[2]) + pt[2];  // NOLINT
}

template <typename T>
inline void QuaternionRotatePointDouble(const T q[4], const double pt[3], T result[3])
{
  // 'scale' is 1 / norm(q).
  const T scale = T(1) / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  // Make unit-norm version of q.
  const T unit[4] = {
    scale * q[0],
    scale * q[1],
    scale * q[2],
    scale * q[3],
  };

  UnitQuaternionRotatePointDouble(unit, pt, result);
}

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CERES_FUNCTIONS_HPP_
