// Copyright 2021 Tier IV, Inc.
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

#include "extrinsic_map_based_calibrator/grid_search_matching.hpp"

#include "tier4_autoware_utils/math/unit_conversion.hpp"
#include <omp.h>

namespace extrinsic_map_base_calibrator
{
GridSearchMatching::GridSearchMatching() {}

bool GridSearchMatching::executeGridSearchMatching(
  const PointCloudT::Ptr & map_pointcloud, const PointCloudT::Ptr & sensor_pointcloud)
{
  if (map_pointcloud == 0 || map_pointcloud->height == 0|| map_pointcloud->width == 0) {
    std::cerr << "Map point cloud is empty" << std::endl;
    return false;
  } else if (sensor_pointcloud == 0 || sensor_pointcloud->height == 0||sensor_pointcloud->width == 0) {
    std::cerr << "Sensor point cloud is empty" << std::endl;
    return false;
  }
  matcher_.setParameter(config_.matching_config);

  static std::chrono::time_point<std::chrono::system_clock> check1_start, check1_end;

  // find out minimum score by grid search
  check1_start = std::chrono::system_clock::now();
  searched_result_ = gridSearch(map_pointcloud, sensor_pointcloud);
  check1_end = std::chrono::system_clock::now();

  std::cout
    << "grid search time "
    << std::chrono::duration_cast<std::chrono::milliseconds>(check1_end - check1_start).count() /
         1000.0
    << std::endl;

  // calculate minimum fitness score
  PointCloudT::Ptr searched_pointcloud(new PointCloudT);
  pcl::transformPointCloud(
    *sensor_pointcloud, *searched_pointcloud, searched_result_.transformation_matrix);
  rematched_result_ = matcher_.ICPMatching(map_pointcloud, searched_pointcloud);

  return true;
}

matchingResult GridSearchMatching::gridSearch(
  const PointCloudT::Ptr & map_pointcloud, const PointCloudT::Ptr & sensor_pointcloud)
{
  std::vector<double> x_elements =
    generateSearchElement(config_.x_range_min_, config_.x_range_max_, config_.x_resolution_);
  std::vector<double> y_elements =
    generateSearchElement(config_.y_range_min_, config_.y_range_max_, config_.y_resolution_);
  std::vector<double> z_elements =
    generateSearchElement(config_.z_range_min_, config_.z_range_max_, config_.z_resolution_);
  std::vector<double> roll_elements =
    generateSearchElement(config_.roll_range_min_, config_.roll_range_max_, config_.roll_resolution_);
  std::vector<double> pitch_elements =
    generateSearchElement(config_.pitch_range_min_, config_.pitch_range_max_, config_.pitch_resolution_);
  std::vector<double> yaw_elements =
    generateSearchElement(config_.yaw_range_min_, config_.yaw_range_max_, config_.yaw_resolution_);

  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_map(new pcl::search::KdTree<pcl::PointXYZ>);
  tree_map->setInputCloud(map_pointcloud);
  matchingResult min_score_result;
  min_score_result.score = DBL_MAX;
  #pragma omp parallel for
  for (const auto & x_element: x_elements) {
    #pragma omp parallel for
    for (const auto & y_element: y_elements) {
      #pragma omp parallel for
      for  (const auto & z_element: z_elements) {
        #pragma omp parallel for
        for  (const auto & roll_element: roll_elements) {
          #pragma omp parallel for
          for  (const auto & pitch_element: pitch_elements) {
            #pragma omp parallel for
            for  (const auto & yaw_element: yaw_elements) {
              Eigen::Matrix4d transformation_score_matrix = getMatrix4d(x_element, y_element, z_element, roll_element, pitch_element, yaw_element);
              PointCloudT::Ptr translated_cloud(new PointCloudT);
              pcl::transformPointCloud(
                *sensor_pointcloud, *translated_cloud, transformation_score_matrix);

              pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_sensor(new pcl::search::KdTree<pcl::PointXYZ>);
              tree_sensor->setInputCloud(translated_cloud);
              double tmp_score =
                matcher_.getFitnessScore(map_pointcloud, translated_cloud, tree_map, tree_sensor);
              #pragma omp critical (score)
              {
                if (tmp_score < min_score_result.score) {
                  min_score_result.score = tmp_score;
                  min_score_result.transformation_matrix = transformation_score_matrix;
                }
              }
            }
          }
        }
      }
    }
  }
  return min_score_result;
}

Eigen::Matrix4d GridSearchMatching::getMatrix4d(const double & x, const double & y, const double & z,
  const double & roll, const double & pitch, const double & yaw)
{
  Eigen::Matrix4d transformation_roll = Eigen::Matrix4d::Identity();
  double phi = tier4_autoware_utils::deg2rad(roll);
  transformation_roll(1, 1) = std::cos(phi);
  transformation_roll(1, 2) = -sin(phi);
  transformation_roll(2, 1) = sin(phi);
  transformation_roll(2, 2) = std::cos(phi);

  Eigen::Matrix4d transformation_pitch = Eigen::Matrix4d::Identity();
  double theta = tier4_autoware_utils::deg2rad(pitch);
  transformation_pitch(0, 0) = std::cos(theta);
  transformation_pitch(0, 2) = sin(theta);
  transformation_pitch(2, 0) = -sin(theta);
  transformation_pitch(2, 2) = std::cos(theta);

  Eigen::Matrix4d transformation_yaw = Eigen::Matrix4d::Identity();
  double psi = tier4_autoware_utils::deg2rad(yaw);
  transformation_yaw(0, 0) = std::cos(psi);
  transformation_yaw(0, 1) = -sin(psi);
  transformation_yaw(1, 0) = sin(psi);
  transformation_yaw(1, 1) = std::cos(psi);

  Eigen::Matrix4d transformation_score_matrix = Eigen::Matrix4d::Identity();
  transformation_score_matrix = transformation_yaw * transformation_pitch * transformation_roll;
  transformation_score_matrix(0, 3) = x;
  transformation_score_matrix(1, 3) = y;
  transformation_score_matrix(2, 3) = z;

  return transformation_score_matrix;
}

std::vector<double> GridSearchMatching::generateSearchElement(
  const double & range_min, const double & range_max, const double & resolution)
{
  std::vector<double> parameters;
  double abs = std::fabs(range_max - range_min);
  int num = static_cast<int>(abs / resolution + 1.0);
  parameters.resize(num);
  for (int i = 0; i < num; ++i) {
    parameters.at(i) = range_min + (resolution * i);
  }

  return parameters;
}

}  // namespace extrinsic_map_base_calibrator
