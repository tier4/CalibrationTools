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

#ifndef EXTRINSIC_MAP_BASED_CALIBRATOR__POINTCLOUD_MATCHER_HPP_
#define EXTRINSIC_MAP_BASED_CALIBRATOR__POINTCLOUD_MATCHER_HPP_

#include <string>
#include <iostream>
#include <memory>
#include <vector>

#include "pcl/PCLPointCloud2.h"
#include "pcl/point_types.h"
#include "pcl/registration/gicp.h"
#include "pcl_ros/transforms.hpp"
#include "pcl/filters/extract_indices.h"

using PointCloudT = pcl::PointCloud<pcl::PointXYZ>;

namespace extrinsic_map_base_calibrator
{

struct matchingResult
{
  Eigen::Matrix4d transformation_matrix;
  double score;
};

struct MatchingConfig{
  int maximum_iteration_;
  double max_correspondence_distance;
  double transformation_epsilon;
  double euclidean_fitness_epsilon;
};

class PointCloudMatcher
{

public:
  explicit PointCloudMatcher()
  {
  };

  void setParameter(MatchingConfig & config)
  {
    config_ = config;
  };

  matchingResult ICPMatching(const PointCloudT::Ptr & map_pointCloud,
    const PointCloudT::Ptr & sensor_pointCloud)
    {
      // set icp input cloud and params
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
      tree1->setInputCloud(map_pointCloud);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
      tree2->setInputCloud(sensor_pointCloud);
      icp.setSearchMethodTarget(tree1);
      icp.setSearchMethodSource(tree2);
      icp.setInputTarget(map_pointCloud);
      icp.setInputSource(sensor_pointCloud);
      icp.setMaximumIterations(config_.maximum_iteration_); //The maximum number of iterations
      icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
      icp.setTransformationEpsilon(config_.transformation_epsilon);
      icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
      // icp align
      PointCloudT Final;
      // std::cout << "icp matching start" << std::endl;
      icp.align(Final);
      // std::cout << "matching score is " << icp.getFitnessScore() << std::endl;
      // std::cout << "icp matching end" << std::endl;

      // set matching result
      matchingResult result;
      result.transformation_matrix = icp.getFinalTransformation().cast<double>();
      result.score = icp.getFitnessScore();
      return result;
  };

  matchingResult ICPMatching(const PointCloudT::Ptr & map_pointCloud,
    const PointCloudT::Ptr & sensor_pointCloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree_map,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree_sensor)
    {
      // set icp input cloud and params
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setSearchMethodTarget(tree_map);
      icp.setSearchMethodSource(tree_sensor);
      icp.setInputTarget(map_pointCloud);
      icp.setInputSource(sensor_pointCloud);
      icp.setMaximumIterations(config_.maximum_iteration_); //The maximum number of iterations
      icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
      icp.setTransformationEpsilon(config_.transformation_epsilon);
      icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
      // icp align
      PointCloudT Final;
      // std::cout << "icp matching start" << std::endl;
      icp.align(Final);
      // std::cout << "matching score is " << icp.getFitnessScore() << std::endl;

      // set matching result
      matchingResult result;
      result.transformation_matrix = icp.getFinalTransformation().cast<double>();
      result.score = icp.getFitnessScore();
      return result;
    };

    double getFitnessScore(const PointCloudT::Ptr & map_pointCloud,
    const PointCloudT::Ptr & sensor_pointCloud, pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree_map,
    pcl::search::KdTree<pcl::PointXYZ>::Ptr &tree_sensor)
    {
      pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
      icp.setSearchMethodTarget(tree_map);
      icp.setSearchMethodSource(tree_sensor);
      icp.setInputTarget(map_pointCloud);
      icp.setInputSource(sensor_pointCloud);
      icp.setMaximumIterations(config_.maximum_iteration_); //The maximum number of iterations
      icp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
      icp.setTransformationEpsilon(config_.transformation_epsilon);
      icp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
      return icp.getFitnessScore();
    };

    matchingResult GICPMatching(const PointCloudT::Ptr & map_pointCloud,
    const PointCloudT::Ptr & sensor_pointCloud)
    {
      pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
      tree1->setInputCloud(sensor_pointCloud);
      tree2->setInputCloud(map_pointCloud);
      gicp.setSearchMethodSource(tree1);
      gicp.setSearchMethodTarget(tree2);
      gicp.setInputSource(sensor_pointCloud);
      gicp.setInputTarget(map_pointCloud);
      gicp.setMaximumIterations(config_.maximum_iteration_); //The maximum number of iterations
      gicp.setMaxCorrespondenceDistance(config_.max_correspondence_distance);
      gicp.setTransformationEpsilon(config_.transformation_epsilon);
      gicp.setEuclideanFitnessEpsilon(config_.euclidean_fitness_epsilon);
      PointCloudT::Ptr Final(new PointCloudT);

      gicp.align(*Final);
      // set matching result
      matchingResult result;
      result.transformation_matrix = gicp.getFinalTransformation().cast<double>();
      result.score = gicp.getFitnessScore();
      return result;
    };

private:
    MatchingConfig config_;

};

}  // namespace extrinsic_map_base_calibrator
#endif  // EXTRINSIC_MAP_BASED_CALIBRATOR__POINTCLOUD_MATCHER_HPP_
