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

#include <Eigen/Dense>
#include <autoware/universe_utils/geometry/geometry.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tier4_ground_plane_utils/ground_plane_utils.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/pca.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

namespace tier4_ground_plane_utils
{

std::tuple<bool, Eigen::Vector4d, pcl::PointCloud<PointType>::Ptr> extractGroundPlane(
  pcl::PointCloud<PointType>::Ptr & pointcloud, const GroundPlaneExtractorParameters & parameters,
  std::vector<Eigen::Vector4d> & outlier_models)
{
  Eigen::Vector4d model;
  pcl::PointCloud<PointType>::Ptr inliers_pointcloud(new pcl::PointCloud<PointType>);

  if (parameters.use_crop_box_filter_) {
    pcl::CropBox<PointType> boxFilter;
    boxFilter.setMin(Eigen::Vector4f(
      parameters.crop_box_min_x_, parameters.crop_box_min_y_, parameters.crop_box_min_z_, 1.0));
    boxFilter.setMax(Eigen::Vector4f(
      parameters.crop_box_max_x_, parameters.crop_box_max_y_, parameters.crop_box_max_z_, 1.0));
    boxFilter.setInputCloud(pointcloud);
    boxFilter.filter(*pointcloud);
  }

  std::vector<pcl::ModelCoefficients> models;
  Eigen::Vector3f rough_normal;

  if (parameters.use_pca_rough_normal_) {
    // Obtain an idea of the ground plane using PCA
    // under the assumption that the axis with less variance will be the ground plane normal
    pcl::PCA<PointType> pca;
    pca.setInputCloud(pointcloud);
    Eigen::MatrixXf vectors = pca.getEigenVectors();
    rough_normal = vectors.col(2);
  } else {
    rough_normal = (parameters.initial_base_to_lidar_transform_.inverse().rotation() *
                    Eigen::Vector3d(0.0, 0.0, 1.0))
                     .cast<float>();
  }

  if (parameters.verbose_) {
    RCLCPP_INFO(
      rclcpp::get_logger("ground_plane_utils"), "Rough plane normal. x=%.3f, y=%.3f, z=%.3f",
      rough_normal.x(), rough_normal.y(), rough_normal.z());
  }

  // Use RANSAC iteratively until we find the ground plane
  // Since walls can have more points, we filter using the PCA-based hypothesis
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;
  pcl::ExtractIndices<PointType> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(parameters.max_inlier_distance_);
  seg.setMaxIterations(parameters.max_iterations_);

  pcl::PointCloud<PointType>::Ptr iteration_cloud = pointcloud;
  int iteration_cloud_size = iteration_cloud->height * iteration_cloud->width;

  while (iteration_cloud_size > parameters.min_plane_points_) {
    seg.setInputCloud(iteration_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
      if (parameters.verbose_) {
        RCLCPP_WARN(rclcpp::get_logger("ground_plane_utils"), "No plane found in the pointcloud");
      }

      break;
    }

    Eigen::Vector3f normal(
      coefficients->values[0], coefficients->values[1], coefficients->values[2]);
    float cos_distance = 1.0 - std::abs(rough_normal.dot(normal));

    model = Eigen::Vector4d(
      coefficients->values[0], coefficients->values[1], coefficients->values[2],
      coefficients->values[3]);

    int inlier_size = static_cast<int>(inliers->indices.size());
    double inlier_percentage = 100.0 * inlier_size / pointcloud->size();

    if (
      inlier_size > parameters.min_plane_points_ &&
      inlier_percentage > parameters.min_plane_points_percentage_ &&
      cos_distance < parameters.max_cos_distance_) {
      if (parameters.verbose_) {
        RCLCPP_INFO(
          rclcpp::get_logger("ground_plane_utils"), "Plane found: inliers=%ld (%.3f)",
          inliers->indices.size(), inlier_percentage);
        RCLCPP_INFO(
          rclcpp::get_logger("ground_plane_utils"), "Plane model. a=%.3f, b=%.3f, c=%.3f, d=%.3f",
          model(0), model(1), model(2), model(3));
        RCLCPP_INFO(
          rclcpp::get_logger("ground_plane_utils"), "Cos distance: %.3f / %.3f", cos_distance,
          parameters.max_cos_distance_);
      }

      // Extract the ground plane inliers
      extract.setInputCloud(iteration_cloud);
      extract.setIndices(inliers);
      extract.setNegative(false);
      extract.filter(*inliers_pointcloud);

      return std::make_tuple(true, model, inliers_pointcloud);
    } else {
      if (parameters.remove_outliers_) {
        bool accept = true;

        for (const auto & outlier_model : outlier_models) {
          Eigen::Vector3f outlier_normal(outlier_model.x(), outlier_model.y(), outlier_model.z());
          float cos_distance = 1.0 - std::abs(outlier_normal.dot(normal));

          if (
            cos_distance < parameters.max_cos_distance_ &&
            std::abs(outlier_model.w() - model.w()) < parameters.remove_outlier_tolerance_) {
            accept = false;
          }
        }

        if (accept) {
          outlier_models.push_back(model);
          RCLCPP_INFO(
            rclcpp::get_logger("ground_plane_utils"),
            "New outlier model: a=%.3f, b=%.3f, c=%.3f, d=%.3f", model(0), model(1), model(2),
            model(3));
        }
      }

      if (parameters.verbose_) {
        RCLCPP_INFO(
          rclcpp::get_logger("ground_plane_utils"),
          "Iteration failed. model: a=%.3f, b=%.3f, c=%.3f, d=%.3f inliers=%lu inlier "
          "percentage=%.2f cos_distance=%.2f",
          model(0), model(1), model(2), model(3), inliers->indices.size(), inlier_percentage,
          cos_distance);
      }
    }

    // Extract the inliers from the pointcloud (the detected plane was not the ground plane)
    extract.setInputCloud(iteration_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);

    pcl::PointCloud<PointType> next_cloud;
    extract.filter(next_cloud);

    iteration_cloud->swap(next_cloud);
    iteration_cloud_size = iteration_cloud->height * iteration_cloud->width;
  }
  return std::make_tuple(false, model, inliers_pointcloud);
}

std::pair<Eigen::Vector4d, pcl::PointIndices::Ptr> extractPlane(
  pcl::PointCloud<PointType>::Ptr pointcloud, double max_inlier_distance, int max_iterations)
{
  // cSpell:ignore SACMODEL
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr final_inliers(new pcl::PointIndices);
  pcl::SACSegmentation<PointType> seg;
  pcl::ExtractIndices<PointType> extract;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(max_inlier_distance);
  seg.setMaxIterations(max_iterations);

  seg.setInputCloud(pointcloud);
  seg.segment(*final_inliers, *coefficients);

  Eigen::Vector4d model(
    coefficients->values[0], coefficients->values[1], coefficients->values[2],
    coefficients->values[3]);

  return std::make_pair(model, final_inliers);
}

void evaluateModels(
  const Eigen::Vector4d & initial_model, const Eigen::Vector4d & estimated_model,
  pcl::PointCloud<PointType>::Ptr inliers)
{
  auto model_error =
    [](float a, float b, float c, float d, pcl::PointCloud<PointType>::Ptr pc) -> float {
    assert(std::abs(a * a + b * b + c * c - 1.f) < 1e-5);
    float sum = 0.f;
    for (auto & p : pc->points) {
      sum += std::abs(a * p.x + b * p.y + c * p.z + d);
    }
    return sum / (pc->height * pc->width);
  };

  float initial_model_error =
    model_error(initial_model(0), initial_model(1), initial_model(2), initial_model(3), inliers);

  float estimated_model_error = model_error(
    estimated_model(0), estimated_model(1), estimated_model(2), estimated_model(3), inliers);

  RCLCPP_INFO(
    rclcpp::get_logger("ground_plane_utils"), "Initial calibration error: %3f m",
    initial_model_error);
  RCLCPP_INFO(
    rclcpp::get_logger("ground_plane_utils"), "Estimated calibration error: %3f m",
    estimated_model_error);
}

Eigen::Vector4d poseToPlaneModel(const Eigen::Isometry3d & pose)
{
  Eigen::Vector3d normal_vector_base(
    0.0, 0.0, 1.0);  // We use a +z for the normal of the plane. TODO: confirm if PCL does the same
  Eigen::Vector3d normal_vector_lidar = pose.rotation() * normal_vector_base;

  Eigen::Vector4d model;  // (a, b, c, d) from a*x + b*y + c*z + d = 0
  model(0) = normal_vector_lidar.x();
  model(1) = normal_vector_lidar.y();
  model(2) = normal_vector_lidar.z();
  model(3) = -normal_vector_lidar.dot(pose.translation());

  return model;
}

Eigen::Isometry3d modelPlaneToPose(const Eigen::Vector4d & model)
{
  Eigen::Vector3d n(model(0), model(1), model(2));
  n.normalize();

  Eigen::Vector3d x0 = -n * model(3);

  // To create a real pose we need to invent a basis
  Eigen::Vector3d base_x, base_y, base_z;
  base_z = n;

  Eigen::Vector3d c1 = Eigen::Vector3d(1.0, 0.0, 0.0).cross(n);
  Eigen::Vector3d c2 = Eigen::Vector3d(0.0, 1.0, 0.0).cross(n);
  Eigen::Vector3d c3 = Eigen::Vector3d(0.0, 0.0, 1.0).cross(n);

  // Any non-zero would work but we use the one with the highest norm (there has to be a non zero)
  if (c1.norm() > c2.norm() && c1.norm() > c3.norm()) {
    base_x = c1;
  } else if (c2.norm() > c3.norm()) {
    base_x = c2;
  } else {
    base_x = c3;
  }

  base_y = base_z.cross(base_x);

  Eigen::Matrix3d rot;
  rot.col(0) = base_x.normalized();
  rot.col(1) = base_y.normalized();
  rot.col(2) = base_z.normalized();

  Eigen::Isometry3d pose;
  pose.translation() = x0;
  pose.linear() = rot;

  return pose;
}

Eigen::Isometry3d estimateBaseLidarTransform(
  const Eigen::Isometry3d & initial_base_to_lidar_transform, const Eigen::Vector4d & model)
{
  const Eigen::Isometry3d lidar_to_initial_base_transform =
    initial_base_to_lidar_transform.inverse();
  const Eigen::Isometry3d lidar_to_ground_transform = modelPlaneToPose(model);

  const Eigen::Isometry3d ground_to_initial_base_transform =
    lidar_to_ground_transform.inverse() * lidar_to_initial_base_transform;

  Eigen::Vector3d ground_to_initial_base_projected_translation =
    ground_to_initial_base_transform.translation();
  ground_to_initial_base_projected_translation.z() = 0;

  Eigen::Vector3d ground_to_initial_base_projected_rotation_x =
    ground_to_initial_base_transform.rotation().col(0);
  ground_to_initial_base_projected_rotation_x.z() = 0.0;
  ground_to_initial_base_projected_rotation_x.normalize();

  Eigen::Matrix3d ground_to_initial_base_projected_rotation;
  ground_to_initial_base_projected_rotation.col(2) = Eigen::Vector3d(0.0, 0.0, 1.0);
  ground_to_initial_base_projected_rotation.col(0) = ground_to_initial_base_projected_rotation_x;
  ground_to_initial_base_projected_rotation.col(1) =
    ground_to_initial_base_projected_rotation.col(2).cross(
      ground_to_initial_base_projected_rotation.col(0));

  Eigen::Isometry3d ground_to_estimated_base_transform;
  ground_to_estimated_base_transform.translation() = ground_to_initial_base_projected_translation;
  ground_to_estimated_base_transform.linear() = ground_to_initial_base_projected_rotation;

  return ground_to_estimated_base_transform.inverse() * lidar_to_ground_transform.inverse();
}

pcl::PointCloud<PointType>::Ptr removeOutliers(
  pcl::PointCloud<PointType>::Ptr input_pointcloud, const Eigen::Vector4d & outlier_plane_model,
  double outlier_tolerance)
{
  pcl::ExtractIndices<PointType> extract;
  pcl::PointCloud<PointType>::Ptr inliers(new pcl::PointCloud<PointType>);
  pcl::PointIndices::Ptr outliers(new pcl::PointIndices);

  for (std::size_t index = 0; index < input_pointcloud->size(); index++) {
    const auto & p = input_pointcloud->points[index];
    double error = std::abs(
      outlier_plane_model.x() * p.x + outlier_plane_model.y() * p.y +
      outlier_plane_model.z() * p.z + outlier_plane_model.w());

    if (error < outlier_tolerance) {
      outliers->indices.push_back(index);
    }
  }

  // Extract the inliers from the pointcloud (the detected plane was not the ground plane)
  extract.setInputCloud(input_pointcloud);
  extract.setIndices(outliers);
  extract.setNegative(true);
  extract.filter(*inliers);

  return inliers;
}

geometry_msgs::msg::TransformStamped overwriteXYYawValues(
  const geometry_msgs::msg::TransformStamped & initial_base_lidar_transform_msg,
  const geometry_msgs::msg::TransformStamped & calibrated_base_lidar_transform_msg)
{
  geometry_msgs::msg::TransformStamped msg = calibrated_base_lidar_transform_msg;

  // Overwrite xy
  msg.transform.translation.x = initial_base_lidar_transform_msg.transform.translation.x;
  msg.transform.translation.y = initial_base_lidar_transform_msg.transform.translation.y;

  auto initial_rpy =
    autoware::universe_utils::getRPY(initial_base_lidar_transform_msg.transform.rotation);

  auto calibrated_rpy =
    autoware::universe_utils::getRPY(calibrated_base_lidar_transform_msg.transform.rotation);

  // Overwrite only yaw
  auto output_rpy = calibrated_rpy;
  output_rpy.z = initial_rpy.z;

  msg.transform.rotation =
    autoware::universe_utils::createQuaternionFromRPY(output_rpy.x, output_rpy.y, output_rpy.z);
  return msg;
}

}  // namespace tier4_ground_plane_utils
