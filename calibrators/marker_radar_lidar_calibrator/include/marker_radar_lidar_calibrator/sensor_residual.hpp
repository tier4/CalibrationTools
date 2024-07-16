// Copyright 2024 Tier IV, Inc.
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

#ifndef MARKER_RADAR_LIDAR_CALIBRATOR__SENSOR_RESIDUAL_HPP_
#define MARKER_RADAR_LIDAR_CALIBRATOR__SENSOR_RESIDUAL_HPP_

#include <Eigen/Dense>

namespace marker_radar_lidar_calibrator
{

struct SensorResidual
{
  SensorResidual(const Eigen::Vector4d & radar_point, const Eigen::Vector4d & lidar_point)
  : radar_point_(radar_point), lidar_point_(lidar_point)
  {
  }

  template <class T>
  bool operator()(T const * const params, T * s_residuals) const
  {
    // parameters: x, y, z, pitch, yaw.
    Eigen::Matrix<T, 4, 4> transformation_matrix = Eigen::Matrix<T, 4, 4>::Identity(4, 4);
    Eigen::Matrix<T, 3, 3> rotation_matrix;

    transformation_matrix(0, 3) = T(params[0]);
    transformation_matrix(1, 3) = T(params[1]);
    transformation_matrix(2, 3) = T(params[2]);

    // This rotation matrix is rotate from radar to optimization frames (usually base_link).
    // To avoid make sure that the Y axis does not approaches 90 degrees to avoid gimbal lock.
    rotation_matrix = (Eigen::AngleAxis<T>(T(params[4]), Eigen::Vector3<T>::UnitZ()) *
                       Eigen::AngleAxis<T>(T(params[3]), Eigen::Vector3<T>::UnitY()) *
                       Eigen::AngleAxis<T>(T(0), Eigen::Vector3<T>::UnitX()))
                        .matrix();

    transformation_matrix.block(0, 0, 3, 3) = rotation_matrix;

    Eigen::Map<Eigen::Matrix<T, 3, 1>> residuals(s_residuals);
    Eigen::Matrix<T, 4, 1> residuals4d =
      lidar_point_.cast<T>() - transformation_matrix * radar_point_.cast<T>();
    residuals = residuals4d.block(0, 0, 3, 1);

    return true;
  }

  Eigen::Vector4d radar_point_;
  Eigen::Vector4d lidar_point_;
};

}  // namespace marker_radar_lidar_calibrator

#endif  // MARKER_RADAR_LIDAR_CALIBRATOR__SENSOR_RESIDUAL_HPP_
