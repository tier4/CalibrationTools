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

}  // namespace extrinsic_tag_based_base_calibrator

#endif  // EXTRINSIC_TAG_BASED_BASE_CALIBRATOR__CERES__CERES_FUNCTIONS_HPP_
