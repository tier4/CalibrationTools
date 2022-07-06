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

#include <Eigen/Core>
#include <extrinsic_tag_based_calibrator/brute_force_matcher.hpp>

#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>

bool bruteForceMatcher(
  PointCloudT::Ptr & source, PointCloudT::Ptr & target, double thresh,
  std::vector<int> & source_indexes, std::vector<int> & target_indexes, bool debug)
{
  if (debug) {
    pcl::console::setVerbosityLevel(pcl::console::VERBOSITY_LEVEL::L_VERBOSE);
  }

  PointCloudT::Ptr source_ransac_aligned(new PointCloudT);
  PointCloudT::Ptr source_icp_aligned(new PointCloudT);
  FeatureCloudT::Ptr source_features(new FeatureCloudT);
  FeatureCloudT::Ptr target_features(new FeatureCloudT);

  // Estimate features
  FeatureEstimationT fest;
  fest.setRadiusSearch(500.0);
  fest.setInputCloud(source);
  fest.setInputNormals(source);
  fest.compute(*source_features);
  fest.setInputCloud(target);
  fest.setInputNormals(target);
  fest.compute(*target_features);

  int num_source_points = source->size();
  int num_target_points = source->size();

  if (num_source_points != num_target_points) {
    return false;
  }

  // Perform RANSAC-based alignment
  pcl::SampleConsensusPrerejective<PointNT, PointNT, FeatureT> ransac_align;
  ransac_align.setInputSource(source);
  ransac_align.setSourceFeatures(source_features);
  ransac_align.setInputTarget(target);
  ransac_align.setTargetFeatures(target_features);
  ransac_align.setMaximumIterations(
    10000);  // Due to the problem dimensionality, this shold be high
  ransac_align.setNumberOfSamples(4);
  ransac_align.setCorrespondenceRandomness(5);
  ransac_align.setSimilarityThreshold(0.9f);         // This will reject most hypotheses
  ransac_align.setMaxCorrespondenceDistance(100.f);  // We accept all points as inliers
  ransac_align.setInlierFraction(1.f);
  ransac_align.align(*source_ransac_aligned);

  if (!ransac_align.hasConverged()) {
    pcl::console::print_error("RANSAC aligning did not converge\n");
    if (debug) {
      pcl::io::savePCDFileASCII("ransac_failed_source.pcd", *source);
      pcl::io::savePCDFileASCII("ransac_failed_target.pcd", *target);
    }
    return false;
  }

  // Find RANSAC-aligned correspondances using nearest neighbor
  pcl::CorrespondencesPtr ransac_correspondences(new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointNT, PointNT> ransac_corr_est;
  ransac_corr_est.setInputSource(source_ransac_aligned);
  ransac_corr_est.setInputTarget(target);
  ransac_corr_est.determineCorrespondences(*ransac_correspondences);

  if (debug) {
    pcl::console::print_info("RANSAC correspondences\n");
    for (int i = 0; i < int(ransac_correspondences->size()); ++i) {
      int source_id = (*ransac_correspondences)[i].index_query;
      int target_id = (*ransac_correspondences)[i].index_match;
      double distance = std::sqrt((*ransac_correspondences)[i].distance);
      pcl::console::print_info(
        "%d: source_id=%d target_id=%d distance=%f\n", i, source_id, target_id, distance);
    }
  }

  pcl::IterativeClosestPoint<PointNT, PointNT> icp;
  icp.setInputSource(source_ransac_aligned);
  icp.setInputTarget(target);
  // icp.setMaxCorrespondenceDistance(2*thresh);
  icp.setMaxCorrespondenceDistance(5.f);
  icp.setMaximumIterations(20);
  icp.setTransformationEpsilon(0.0);

  icp.align(*source_icp_aligned);

  if (!icp.hasConverged()) {
    pcl::console::print_error("ICP alining did not converge\n");
    if (debug) {
      pcl::io::savePCDFileASCII("icp_failed_source.pcd", *source);
      pcl::io::savePCDFileASCII("icp_failed_target.pcd", *target);
    }
    return false;
  }

  // Find ICP-aligned correspondances using nearest neighbor
  pcl::CorrespondencesPtr icp_correspondences(new pcl::Correspondences);
  pcl::registration::CorrespondenceEstimation<PointNT, PointNT> icp_corr_est;
  icp_corr_est.setInputSource(source_icp_aligned);
  icp_corr_est.setInputTarget(target);
  icp_corr_est.determineCorrespondences(*icp_correspondences);

  if (debug) {
    pcl::console::print_info("ICP correspondences\n");
    for (int i = 0; i < int(icp_correspondences->size()); ++i) {
      int source_id = (*icp_correspondences)[i].index_query;
      int target_id = (*icp_correspondences)[i].index_match;
      double distance = std::sqrt((*icp_correspondences)[i].distance);
      pcl::console::print_info(
        "%d: source_id=%d target_id=%d distance=%f\n", i, source_id, target_id, distance);
    }
  }

  if (debug) {
    // Print results
    pcl::console::print_error("RANSAC aligning results\n");
    Eigen::Matrix4f transformation = ransac_align.getFinalTransformation();
    pcl::console::print_info(
      "    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1),
      transformation(0, 2));
    pcl::console::print_info(
      "R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1),
      transformation(1, 2));
    pcl::console::print_info(
      "    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1),
      transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info(
      "t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3),
      transformation(2, 3));
    pcl::console::print_info("\n");
    pcl::console::print_info("Inliers: %i/%i\n", ransac_align.getInliers().size(), source->size());
    pcl::console::print_info("Initial align fitness: %f\n", ransac_align.getFitnessScore());

    pcl::console::print_error("RANSAC aligning results\n");
    transformation = icp.getFinalTransformation();
    pcl::console::print_info(
      "    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1),
      transformation(0, 2));
    pcl::console::print_info(
      "R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1),
      transformation(1, 2));
    pcl::console::print_info(
      "    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1),
      transformation(2, 2));
    pcl::console::print_info("\n");
    pcl::console::print_info(
      "t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3),
      transformation(2, 3));
    pcl::console::print_info("\n");
    pcl::console::print_info("ICP align fitness: %f\n", icp.getFitnessScore());
  }

  // Check if the correspondances conform a bijection
  std::unordered_map<int, int> map;
  for (auto & correspondence : *icp_correspondences) {
    map[correspondence.index_match] = 1;
  }

  if (static_cast<int>(map.size()) != num_source_points) {
    pcl::console::print_error("Correspondences do not form a bijection\n");
    return false;
  }

  // Check if the ICP aligning satisties the convergence criteria

  for (int i = 0; i < int(icp_correspondences->size()); ++i) {
    int source_id = (*icp_correspondences)[i].index_query;
    int target_id = (*icp_correspondences)[i].index_match;
    double distance = std::sqrt((*icp_correspondences)[i].distance);

    if (distance > thresh) {
      pcl::console::print_error(
        "%d: source_id=%d target_id=%d distance=%f\n", i, source_id, target_id, distance);

      if (debug) {
        pcl::io::savePCDFileASCII("dist_failed_source.pcd", *source);
        pcl::io::savePCDFileASCII("dist_failed_target.pcd", *target);
      }
      return false;
    }
  }

  // Fill the results
  source_indexes.clear();
  target_indexes.clear();

  for (int i = 0; i < int(icp_correspondences->size()); ++i) {
    int source_id = (*icp_correspondences)[i].index_query;
    int target_id = (*icp_correspondences)[i].index_match;

    source_indexes.push_back(source_id);
    target_indexes.push_back(target_id);
  }

  return true;
}
