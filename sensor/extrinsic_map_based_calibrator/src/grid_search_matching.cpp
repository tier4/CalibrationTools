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

namespace extrinsic_map_base_calibrator
{
GridSearchMatching::GridSearchMatching() {}

bool GridSearchMatching::executeGridSearchMatching(
  const PointCloudT::Ptr & map_pointcloud, const PointCloudT::Ptr & sensor_pointcloud)
{
  if (map_pointcloud == 0 || map_pointcloud->height == 0) {
    std::cerr << "Map point cloud is empty" << std::endl;
    return false;
  } else if (sensor_pointcloud == 0 || sensor_pointcloud->height == 0) {
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
  std::vector<double> x_score =
    generateSearchElement(config_.x_range_min_, config_.x_range_max_, config_.x_resolution_);
  std::vector<double> y_score =
    generateSearchElement(config_.y_range_min_, config_.y_range_max_, config_.y_resolution_);
  std::vector<double> yaw_score =
    generateSearchElement(config_.yaw_range_min_, config_.yaw_range_max_, config_.yaw_resolution_);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_map(new pcl::search::KdTree<pcl::PointXYZ>);
  tree_map->setInputCloud(map_pointcloud);
  matchingResult min_score_result;
  min_score_result.score = DBL_MAX;

  for (size_t i = 0; i < x_score.size(); ++i) {
    for (size_t j = 0; j < y_score.size(); ++j) {
      for (size_t k = 0; k < yaw_score.size(); ++k) {
        Eigen::Matrix4d transformation_score_matrix = Eigen::Matrix4d::Identity();

        double theta_score = tier4_autoware_utils::deg2rad(yaw_score.at(k));
        transformation_score_matrix(0, 0) = std::cos(theta_score);
        transformation_score_matrix(0, 1) = -sin(theta_score);
        transformation_score_matrix(1, 0) = sin(theta_score);
        transformation_score_matrix(1, 1) = std::cos(theta_score);
        transformation_score_matrix(0, 3) = x_score.at(i);
        transformation_score_matrix(1, 3) = y_score.at(j);
        transformation_score_matrix(2, 3) = 0.0;

        PointCloudT::Ptr translated_cloud(new PointCloudT);
        pcl::transformPointCloud(
          *sensor_pointcloud, *translated_cloud, transformation_score_matrix);

        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_sensor(new pcl::search::KdTree<pcl::PointXYZ>);
        tree_sensor->setInputCloud(translated_cloud);
        double tmp_score =
          matcher_.getFitnessScore(map_pointcloud, translated_cloud, tree_map, tree_sensor);

        if (tmp_score < min_score_result.score) {
          min_score_result.score = tmp_score;
          min_score_result.transformation_matrix = transformation_score_matrix;
        }
      }
    }
  }
  return min_score_result;
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
