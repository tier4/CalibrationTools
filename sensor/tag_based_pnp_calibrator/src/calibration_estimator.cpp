// Copyright 2023 Tier IV, Inc.
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

#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <tag_based_pnp_calibrator/brute_force_matcher.hpp>
#include <tag_based_pnp_calibrator/calibration_estimator.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl/point_types.h>
#include <tf2/utils.h>

#include <limits>
#include <random>

CalibrationEstimator::CalibrationEstimator()
: min_pnp_pairs_(4),
  min_convergence_time_(5.0),
  max_no_observation_time_(2.0),
  lidartag_convergence_transl_(0.05),
  lidartag_convergence_transl_dot_(0.001),
  lidartag_convergence_rot_(),
  lidartag_convergence_rot_dot_(0.0001),
  lidartag_new_hypothesis_transl_(0.2),
  lidartag_new_hypothesis_rot_(CV_PI * 45 / 180.0),
  lidartag_process_noise_transl_(0.005),
  lidartag_process_noise_transl_dot_(0.005),
  lidartag_process_noise_rot_(CV_PI * 0.5 / 180.0),
  lidartag_process_noise_rot_dot_(CV_PI * 0.5 / 180.0),
  lidartag_measurement_noise_transl_(0.05),
  lidartag_measurement_noise_rot_(CV_PI * 5 / 180.0),
  apriltag_convergence_transl_(0.5),
  apriltag_new_hypothesis_transl_(10.0),
  apriltag_process_noise_transl_(0.5),
  apriltag_measurement_noise_transl_(2.0),
  crossvalidation_reprojection_error_(std::numeric_limits<double>::infinity()),
  valid_(false)
{
}

void CalibrationEstimator::update(const apriltag_msgs::msg::AprilTagDetectionArray & msg)
{
  for (auto & detection : msg.detections) {
    update(detection, rclcpp::Time(msg.header.stamp));
  }
}

void CalibrationEstimator::update(const lidartag_msgs::msg::LidarTagDetectionArray & msg)
{
  for (auto & detection : msg.detections) {
    update(detection, rclcpp::Time(msg.header.stamp));
  }
}

void CalibrationEstimator::update(
  const apriltag_msgs::msg::AprilTagDetection & detection, const rclcpp::Time & stamp)
{
  if (!pinhole_camera_model_.initialized() || tag_sizes_map_.count(detection.id) == 0) {
    return;
  }

  std::vector<cv::Point2d> corners;

  for (auto & c : detection.corners) {
    corners.push_back(cv::Point2d(c.x, c.y));
  }

  // 1) Create a new hypothesis for comparison convenience
  auto new_h =
    std::make_shared<tier4_tag_utils::ApriltagHypothesis>(detection.id, pinhole_camera_model_);
  new_h->setMaxConvergenceThreshold(apriltag_convergence_transl_);
  new_h->setMaxNoObservationTime(max_no_observation_time_);
  new_h->setMeasurementNoise(apriltag_measurement_noise_transl_);
  new_h->setMinConvergenceTime(min_convergence_time_);
  new_h->setNewHypothesisThreshold(apriltag_new_hypothesis_transl_);
  new_h->setProcessNoise(apriltag_process_noise_transl_);
  new_h->setTagSize(tag_sizes_map_[detection.id]);
  new_h->update(corners, stamp);

  // 2) Compare with already converged hypotheses
  cv::Point3d new_center = new_h->getCenter3d();
  for (std::shared_ptr<tier4_tag_utils::ApriltagHypothesis> & h : converged_apriltag_hypotheses_) {
    cv::Point3d center = h->getCenter3d();
    double distance = cv::norm(new_center - center);

    if (distance < new_hypothesis_distance_) {
      return;
    }
  }

  // 3) Compare with active hypotheses (not yet converged)
  for (std::pair<int, std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> h :
       active_apriltag_hypotheses_) {
    if (h.first != detection.id) {
      continue;
    }

    h.second->update(corners, stamp);
    return;
  }

  // 4) Add the new hypotheses to the active list
  active_apriltag_hypotheses_[detection.id] = new_h;
}

void CalibrationEstimator::update(
  const lidartag_msgs::msg::LidarTagDetection & detection, const rclcpp::Time & stamp)
{
  cv::Matx31d translation_cv;
  cv::Matx33d rotation_cv;

  Eigen::Isometry3d pose_eigen;
  tf2::fromMsg(detection.pose, pose_eigen);

  Eigen::Vector3d translation_eigen = pose_eigen.translation();
  Eigen::Matrix3d rotation_eigen = pose_eigen.rotation();

  cv::eigen2cv(translation_eigen, translation_cv);
  cv::eigen2cv(rotation_eigen, rotation_cv);

  // 1) Create a new hypothesis for convenience
  int hypothesis_id = detection.id >= 0 ? detection.id : (-active_lidartag_hypotheses_.size() - 1);
  auto new_h = std::make_shared<tier4_tag_utils::LidartagHypothesis>(hypothesis_id);
  new_h->setMaxNoObservationTime(max_no_observation_time_);
  new_h->setMinConvergenceTime(min_convergence_time_);

  new_h->setMaxConvergenceThreshold(
    lidartag_convergence_transl_, lidartag_convergence_transl_dot_, lidartag_convergence_rot_,
    lidartag_convergence_rot_dot_);
  new_h->setMeasurementNoise(lidartag_measurement_noise_transl_, lidartag_measurement_noise_rot_);
  new_h->setNewHypothesisThreshold(lidartag_new_hypothesis_transl_, lidartag_new_hypothesis_rot_);
  new_h->setProcessNoise(
    lidartag_process_noise_transl_, lidartag_process_noise_transl_dot_, lidartag_process_noise_rot_,
    lidartag_process_noise_rot_dot_);
  new_h->update(translation_cv, rotation_cv, detection.size, stamp);

  // 2) Compare with converged hypotheses
  cv::Point3d new_center = new_h->getCenter();

  for (std::shared_ptr<tier4_tag_utils::LidartagHypothesis> h : converged_lidartag_hypotheses_) {
    cv::Point3d center = h->getCenter();
    double distance = cv::norm(new_center - center);

    if (distance < new_hypothesis_distance_) {
      return;
    }
  }

  for (std::pair<int, std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> h :
       active_lidartag_hypotheses_) {
    if (detection.id >= 0 && h.first != detection.id) {
      continue;
    } else if (
      cv::norm(new_h->getCenter() - h.second->getCenter()) > new_hypothesis_distance_ &&
      detection.id < 0) {
      continue;
    }

    h.second->update(translation_cv, rotation_cv, detection.size, stamp);

    std::vector<cv::Point3d> latest_points = h.second->getLatestPoints();
    std::vector<cv::Point3d> filtered_points = h.second->getFilteredPoints();

    return;
  }

  active_lidartag_hypotheses_[hypothesis_id] = new_h;
}

bool CalibrationEstimator::update(const rclcpp::Time & stamp)
{
  // 1) Update the hypotheses
  std::vector<int> lidartag_delete_ids, apriltag_delete_ids;
  for (std::pair<int, std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> h :
       active_lidartag_hypotheses_) {
    if (!h.second->update(stamp)) {
      lidartag_delete_ids.push_back(h.first);
    }
  }

  for (std::pair<int, std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> h :
       active_apriltag_hypotheses_) {
    if (!h.second->update(stamp)) {
      apriltag_delete_ids.push_back(h.first);
    }
  }

  for (const int & id : lidartag_delete_ids) {
    active_lidartag_hypotheses_.erase(id);
  }

  for (const int & id : apriltag_delete_ids) {
    active_apriltag_hypotheses_.erase(id);
  }

  // 2) Check if all the hypotheses converged
  bool apriltag_to_lidartag_match = true;
  for (std::pair<int, std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> h :
       active_apriltag_hypotheses_) {
    if (!h.second->converged()) {
      return true;
    }

    apriltag_to_lidartag_match &= active_lidartag_hypotheses_.count(h.first) == 1;
  }

  for (std::pair<int, std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> h :
       active_lidartag_hypotheses_) {
    if (
      apriltag_to_lidartag_match && active_apriltag_hypotheses_.count(h.first) == 1 &&
      !h.second->converged()) {
      return true;
    } else if (!apriltag_to_lidartag_match && !h.second->converged()) {
      return true;
    }
  }

  // 3) Add the hypotheses to the converged list
  if (apriltag_to_lidartag_match) {
    for (std::pair<int, std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> h :
         active_apriltag_hypotheses_) {
      auto & h_apriltag = h.second;
      auto & h_lidartag = active_lidartag_hypotheses_[h.first];

      converged_apriltag_hypotheses_.push_back(h_apriltag);
      converged_lidartag_hypotheses_.push_back(h_lidartag);
    }
  } else {
    if (active_apriltag_hypotheses_.size() != active_lidartag_hypotheses_.size()) {
      return true;
    }

    for (std::pair<int, std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> h :
         active_lidartag_hypotheses_) {
      converged_lidartag_hypotheses_.push_back(h.second);
    }

    for (std::pair<int, std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> h :
         active_apriltag_hypotheses_) {
      converged_apriltag_hypotheses_.push_back(h.second);
    }
  }

  active_lidartag_hypotheses_.clear();
  active_apriltag_hypotheses_.clear();

  return true;
}

std::tuple<std::vector<cv::Point3d>, std::vector<cv::Point2d>>
CalibrationEstimator::getCalibrationPoints(bool use_estimated)
{
  bool negative_id = false;

  for (const std::shared_ptr<tier4_tag_utils::LidartagHypothesis> & h :
       converged_lidartag_hypotheses_) {
    if (h->getId() < 0) {
      negative_id = true;
      break;
    }
  }

  if (negative_id) {
    return getCalibrationPointsIdless(use_estimated);
  } else {
    return getCalibrationPointsIdBased(use_estimated);
  }
}

std::tuple<std::vector<cv::Point3d>, std::vector<cv::Point2d>>
CalibrationEstimator::getCalibrationPointsIdBased(bool use_estimated)
{
  assert(converged_lidartag_hypotheses_.size() == converged_apriltag_hypotheses_.size());
  std::vector<cv::Point3d> lidartag_object_points;
  std::vector<cv::Point2d> apriltag_image_points;

  for (std::size_t i = 0; i < converged_lidartag_hypotheses_.size(); ++i) {
    std::shared_ptr<tier4_tag_utils::LidartagHypothesis> & lidartag_h =
      converged_lidartag_hypotheses_[i];
    std::shared_ptr<tier4_tag_utils::ApriltagHypothesis> & apriltag_h =
      converged_apriltag_hypotheses_[i];

    std::vector<cv::Point3d> h_lidartag_object_points;
    std::vector<cv::Point2d> h_apriltag_image_points;

    if (use_estimated) {
      h_lidartag_object_points = lidartag_h->getFilteredPoints();
      h_apriltag_image_points = apriltag_h->getFilteredPoints2d();
    } else {
      h_lidartag_object_points = lidartag_h->getLatestPoints();
      h_apriltag_image_points = apriltag_h->getLatestPoints2d();
    }

    lidartag_object_points.insert(
      lidartag_object_points.end(), h_lidartag_object_points.begin(),
      h_lidartag_object_points.end());
    apriltag_image_points.insert(
      apriltag_image_points.end(), h_apriltag_image_points.begin(), h_apriltag_image_points.end());
  }

  return std::make_tuple(lidartag_object_points, apriltag_image_points);
}

std::tuple<std::vector<cv::Point3d>, std::vector<cv::Point2d>>
CalibrationEstimator::getCalibrationPointsIdless(bool use_estimated)
{
  std::vector<cv::Point3d> object_points;
  std::vector<cv::Point2d> image_points;

  std::vector<cv::Point2d> apriltag_image_points;
  std::vector<cv::Point3d> apriltag_object_points;
  std::vector<cv::Point3d> apriltag_object_normals;
  std::vector<cv::Point3d> lidartag_object_points;
  std::vector<cv::Point3d> lidartag_object_normals;

  double min_board_size = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < converged_lidartag_hypotheses_.size(); ++i) {
    std::shared_ptr<tier4_tag_utils::LidartagHypothesis> & lidartag_h =
      converged_lidartag_hypotheses_[i];
    std::shared_ptr<tier4_tag_utils::ApriltagHypothesis> & apriltag_h =
      converged_apriltag_hypotheses_[i];

    double board_size = tag_sizes_map_[apriltag_h->getId()];
    min_board_size = std::min(min_board_size, board_size);

    std::vector<cv::Point3d> apriltag_template_points = {
      cv::Point3d(-0.5 * board_size, 0.5 * board_size, 0.0),
      cv::Point3d(0.5 * board_size, 0.5 * board_size, 0.0),
      cv::Point3d(0.5 * board_size, -0.5 * board_size, 0.0),
      cv::Point3d(-0.5 * board_size, -0.5 * board_size, 0.0)};

    std::vector<cv::Point3d> h_lidartag_object_points;
    std::vector<cv::Point3d> h_apriltag_object_points;
    std::vector<cv::Point2d> h_apriltag_image_points;

    std::vector<cv::Point3d> h_lidartag_object_normals;
    std::vector<cv::Point3d> h_apriltag_object_normals;

    if (use_estimated) {
      h_lidartag_object_points = lidartag_h->getFilteredPoints();
      h_apriltag_image_points = apriltag_h->getFilteredPoints2d();
      h_apriltag_object_points = apriltag_h->getFilteredPoints3d();
    } else {
      h_lidartag_object_points = lidartag_h->getLatestPoints();
      h_apriltag_image_points = apriltag_h->getLatestPoints2d();
      h_apriltag_object_points = apriltag_h->getLatestPoints3d();
    }

    // Calculate the normals
    assert(h_lidartag_object_points.size() == 4);
    assert(h_apriltag_object_points.size() == 4);

    auto calculate_normals_form_corners = [](std::vector<cv::Point3d> & corners) {
      cv::Point3d center = 0.25 * (corners[0] + corners[1] + corners[2] + corners[3]);
      cv::Point3d u = corners[2] - corners[0];
      cv::Point3d v = corners[3] - corners[1];
      cv::Point3d n = u.cross(v);
      n = n.dot(center) > 0.0 ? -n : n;
      n /= cv::norm(n);
      return std::vector<cv::Point3d>({n, n, n, n});
    };

    h_apriltag_object_normals = calculate_normals_form_corners(h_apriltag_object_points);
    h_lidartag_object_normals = calculate_normals_form_corners(h_lidartag_object_points);

    apriltag_object_normals.insert(
      apriltag_object_normals.end(), h_apriltag_object_normals.begin(),
      h_apriltag_object_normals.end());
    lidartag_object_normals.insert(
      lidartag_object_normals.end(), h_lidartag_object_normals.begin(),
      h_lidartag_object_normals.end());
    apriltag_object_points.insert(
      apriltag_object_points.end(), h_apriltag_object_points.begin(),
      h_apriltag_object_points.end());
    lidartag_object_points.insert(
      lidartag_object_points.end(), h_lidartag_object_points.begin(),
      h_lidartag_object_points.end());
    apriltag_image_points.insert(
      apriltag_image_points.end(), h_apriltag_image_points.begin(), h_apriltag_image_points.end());
  }

  int num_points = apriltag_image_points.size();

  pcl::PointCloud<pcl::PointNormal>::Ptr lidartag_cloud(
    new pcl::PointCloud<pcl::PointNormal>(num_points, 1));
  pcl::PointCloud<pcl::PointNormal>::Ptr apriltag_cloud(
    new pcl::PointCloud<pcl::PointNormal>(num_points, 1));

  // Fill in the CloudIn data
  for (int i = 0; i < num_points; ++i) {
    lidartag_cloud->points[i].x = lidartag_object_points[i].x;
    lidartag_cloud->points[i].y = lidartag_object_points[i].y;
    lidartag_cloud->points[i].z = lidartag_object_points[i].z;
    lidartag_cloud->points[i].normal_x = lidartag_object_normals[i].x;
    lidartag_cloud->points[i].normal_y = lidartag_object_normals[i].y;
    lidartag_cloud->points[i].normal_z = lidartag_object_normals[i].z;

    apriltag_cloud->points[i].x = apriltag_object_points[i].x;
    apriltag_cloud->points[i].y = apriltag_object_points[i].y;
    apriltag_cloud->points[i].z = apriltag_object_points[i].z;
    apriltag_cloud->points[i].normal_x = apriltag_object_normals[i].x;
    apriltag_cloud->points[i].normal_y = apriltag_object_normals[i].y;
    apriltag_cloud->points[i].normal_z = apriltag_object_normals[i].z;
  }

  std::vector<int> lidartag_indexes, apriltag_indexes;
  double thresh = min_board_size;  // error that assures us that no corner is mismatched

  if (
    apriltag_cloud->size() != lidartag_cloud->size() ||
    static_cast<int>(apriltag_cloud->size()) < min_pnp_pairs_) {
    return std::make_tuple(object_points, image_points);
  }

  if (!bruteForceMatcher(
        apriltag_cloud, lidartag_cloud, thresh, apriltag_indexes, lidartag_indexes, false)) {
    return std::make_tuple(object_points, image_points);
  }

  assert(apriltag_indexes.size() == lidartag_indexes.size());

  for (std::size_t i = 0; i < apriltag_indexes.size(); i++) {
    object_points.push_back(lidartag_object_points[lidartag_indexes[i]]);
    image_points.push_back(apriltag_image_points[apriltag_indexes[i]]);
  }

  return std::make_tuple(object_points, image_points);
}

bool CalibrationEstimator::calibrate()
{
  auto [observation_object_points, observation_image_points] = getCalibrationPoints(false);
  auto [estimated_object_points, estimated_image_points] = getCalibrationPoints(true);

  auto [observation_status, observation_translation_vector, observation_rotation_matrix] =
    calibrate(observation_object_points, observation_image_points);
  auto [estimation_status, hypothesis_translation_vector, hypothesis_rotation_matrix] =
    calibrate(estimated_object_points, estimated_image_points);

  bool status = observation_status && estimation_status;
  valid_ |= status;

  if (status) {
    observation_translation_vector_ = observation_translation_vector;
    observation_rotation_matrix_ = observation_rotation_matrix;
    hypothesis_translation_vector_ = hypothesis_translation_vector;
    hypothesis_rotation_matrix_ = hypothesis_rotation_matrix;
  }

  computeCrossValidationReprojectionError(estimated_object_points, estimated_image_points);

  return status;
}

std::tuple<bool, cv::Matx31d, cv::Matx33d> CalibrationEstimator::calibrate(
  const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points)
{
  cv::Matx31d translation_vector;
  cv::Matx33d rotation_matrix;

  if (
    object_points.size() != image_points.size() ||
    static_cast<int>(object_points.size()) < min_pnp_pairs_) {
    return std::tuple<bool, cv::Matx31d, cv::Matx33d>(false, translation_vector, rotation_matrix);
  }

  auto camera_intrinsics = pinhole_camera_model_.intrinsicMatrix();
  auto distortion_coeffs = pinhole_camera_model_.distortionCoeffs();

  cv::Mat rvec, tvec;

  bool success = cv::solvePnP(
    object_points, image_points, camera_intrinsics, distortion_coeffs, rvec, tvec, false,
    cv::SOLVEPNP_SQPNP);

  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("calibration_estimator"), "PNP failed");
    return std::tuple<bool, cv::Matx31d, cv::Matx33d>(false, translation_vector, rotation_matrix);
  }

  translation_vector = tvec;
  cv::Rodrigues(rvec, rotation_matrix);

  return std::tuple<bool, cv::Matx31d, cv::Matx33d>(true, translation_vector, rotation_matrix);
}

tf2::Transform CalibrationEstimator::toTf2(
  const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix) const
{
  // cv -> tf2
  tf2::Vector3 tf2_trans(
    translation_vector(0, 0), translation_vector(0, 1), translation_vector(0, 2));

  tf2::Matrix3x3 tf2_rot_matrix(
    rotation_matrix(0, 0), rotation_matrix(0, 1), rotation_matrix(0, 2), rotation_matrix(1, 0),
    rotation_matrix(1, 1), rotation_matrix(1, 2), rotation_matrix(2, 0), rotation_matrix(2, 1),
    rotation_matrix(2, 2));

  return tf2::Transform(tf2_rot_matrix, tf2_trans);
}

void CalibrationEstimator::computeCrossValidationReprojectionError(
  const std::vector<cv::Point3d> & object_points, const std::vector<cv::Point2d> & image_points)
{
  // Iterate a number of times
  // Permutate the image object
  // Separate into train and test
  constexpr int trials = 30;

  std::vector<int> indexes(object_points.size());
  std::iota(indexes.begin(), indexes.end(), 0);

  std::mt19937 g(0);  // Same seed for all calls

  std::size_t training_size = crossvalidation_training_ratio_ * object_points.size();

  if (static_cast<int>(training_size) < min_pnp_pairs_) {
    return;
  }

  double error = 0.0;

  for (int trial = 0; trial < trials; trial++) {
    std::shuffle(indexes.begin(), indexes.end(), g);

    std::vector<cv::Point3d> training_object_points, test_object_points;
    std::vector<cv::Point2d> training_image_points, test_image_points;

    for (std::size_t i = 0; i < indexes.size(); i++) {
      if (i < training_size) {
        training_object_points.push_back(object_points[indexes[i]]);
        training_image_points.push_back(image_points[indexes[i]]);
      } else {
        test_object_points.push_back(object_points[indexes[i]]);
        test_image_points.push_back(image_points[indexes[i]]);
      }
    }

    std::vector<cv::Point2d> eval_projected_points;

    [[maybe_unused]] auto [status, iter_translation_vector, iter_rotation_matrix] =
      calibrate(training_object_points, training_image_points);

    cv::Matx31d iter_rvec;
    cv::Rodrigues(iter_rotation_matrix, iter_rvec);

    cv::projectPoints(
      test_object_points, iter_rvec, iter_translation_vector,
      pinhole_camera_model_.intrinsicMatrix(), pinhole_camera_model_.distortionCoeffs(),
      eval_projected_points);

    double reprojection_error = 0.0;
    for (std::size_t i = 0; i < test_image_points.size(); i++) {
      double dist = cv::norm(test_image_points[i] - eval_projected_points[i]);
      reprojection_error += dist;
    }

    reprojection_error /= test_image_points.size();
    error += reprojection_error;
  }

  crossvalidation_reprojection_error_ = error / trials;
}

bool CalibrationEstimator::converged() const
{
  return getCurrentCalibrationPairsNumber() >= convergence_min_pairs_ &&
         getCalibrationCoveragePercentage() >= convergence_min_area_percentage_;
}

bool CalibrationEstimator::valid() const { return valid_; }

std::vector<std::shared_ptr<tier4_tag_utils::LidartagHypothesis>>
CalibrationEstimator::getActiveLidartagHypotheses() const
{
  std::vector<std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> v;

  for (std::pair<int, std::shared_ptr<tier4_tag_utils::LidartagHypothesis>> h :
       active_lidartag_hypotheses_) {
    v.push_back(h.second);
  }

  return v;
}

std::vector<std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>>
CalibrationEstimator::getActiveApriltagHypotheses() const
{
  std::vector<std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> v;

  for (std::pair<int, std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>> h :
       active_apriltag_hypotheses_) {
    v.push_back(h.second);
  }

  return v;
}

std::vector<std::shared_ptr<tier4_tag_utils::LidartagHypothesis>>
CalibrationEstimator::getConvergedLidartagHypotheses() const
{
  return converged_lidartag_hypotheses_;
}

std::vector<std::shared_ptr<tier4_tag_utils::ApriltagHypothesis>>
CalibrationEstimator::getConvergedApriltagHypotheses() const
{
  return converged_apriltag_hypotheses_;
}

void CalibrationEstimator::setCameraModel(const sensor_msgs::msg::CameraInfo & camera_info)
{
  pinhole_camera_model_.fromCameraInfo(camera_info);
}

tf2::Transform CalibrationEstimator::getCurrentPoseAsTF() const
{
  return toTf2(observation_translation_vector_, observation_rotation_matrix_);
}

std::tuple<cv::Matx31d, cv::Matx33d> CalibrationEstimator::getCurrentPose() const
{
  return std::tuple<cv::Matx31d, cv::Matx33d>(
    observation_translation_vector_, observation_rotation_matrix_);
}

tf2::Transform CalibrationEstimator::getFilteredPoseAsTF() const
{
  return toTf2(hypothesis_translation_vector_, hypothesis_rotation_matrix_);
}

std::tuple<cv::Matx31d, cv::Matx33d> CalibrationEstimator::getFilteredPose() const
{
  return std::tuple<cv::Matx31d, cv::Matx33d>(
    hypothesis_translation_vector_, hypothesis_rotation_matrix_);
}

void CalibrationEstimator::setCrossvalidationTrainingRatio(double ratio)
{
  crossvalidation_training_ratio_ = ratio;
}

void CalibrationEstimator::setCalibrationConvergenceCriteria(
  int min_pairs, double min_area_percentage)
{
  convergence_min_pairs_ = min_pairs;
  convergence_min_area_percentage_ = min_area_percentage;
}

void CalibrationEstimator::setMinPnpPairs(int min_pairs) { min_pnp_pairs_ = min_pairs; }

void CalibrationEstimator::setMinConvergenceTime(double convergence_time)
{
  min_convergence_time_ = convergence_time;
}

void CalibrationEstimator::setMaxNoObservationTime(double time) { max_no_observation_time_ = time; }

void CalibrationEstimator::setNewHypothesisDistance(double distance)
{
  new_hypothesis_distance_ = distance;
}

void CalibrationEstimator::setTagSizes(
  std::vector<int64_t> & tag_ids, std::vector<double> & tag_sizes)
{
  assert(tag_ids.size() == tag_sizes.size());

  for (std::size_t i = 0; i < tag_ids.size(); ++i) {
    tag_sizes_map_[tag_ids[i]] = tag_sizes[i];
  }
}

void CalibrationEstimator::setLidartagMaxConvergenceThreshold(
  double transl, double transl_dot, double rot, double rot_dot)
{
  lidartag_convergence_transl_ = transl;
  lidartag_convergence_transl_dot_ = transl_dot;
  lidartag_convergence_rot_ = CV_PI * rot / 180.0;
  lidartag_convergence_rot_dot_ = CV_PI * rot_dot / 180.0;
}

void CalibrationEstimator::setLidartagNewHypothesisThreshold(double max_transl, double max_rot)
{
  lidartag_new_hypothesis_transl_ = max_transl;
  lidartag_new_hypothesis_rot_ = CV_PI * max_rot / 180.0;
}

void CalibrationEstimator::setLidartagMeasurementNoise(double transl, double rot)
{
  lidartag_measurement_noise_transl_ = transl;
  lidartag_measurement_noise_rot_ = CV_PI * rot / 180.0;
}

void CalibrationEstimator::setLidartagProcessNoise(
  double transl, double transl_dot, double rot, double rot_dot)
{
  lidartag_process_noise_transl_ = transl;
  lidartag_process_noise_transl_dot_ = transl_dot;
  lidartag_process_noise_rot_ = CV_PI * rot / 180.0;
  lidartag_process_noise_rot_dot_ = CV_PI * rot_dot / 180.0;
}

void CalibrationEstimator::setApriltagMaxConvergenceThreshold(double transl)
{
  apriltag_convergence_transl_ = transl;
}

void CalibrationEstimator::setApriltagNewHypothesisThreshold(double max_transl)
{
  apriltag_new_hypothesis_transl_ = max_transl;
}

void CalibrationEstimator::setApriltagMeasurementNoise(double transl)
{
  apriltag_measurement_noise_transl_ = transl;
}

void CalibrationEstimator::setApriltagProcessNoise(double transl)
{
  apriltag_process_noise_transl_ = transl;
}

double CalibrationEstimator::getNewHypothesisDistance() const { return new_hypothesis_distance_; }

double CalibrationEstimator::getCalibrationCoveragePercentage() const
{
  double width = pinhole_camera_model_.fullResolution().width;

  cv::Point3d v_left =
    pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(0, pinhole_camera_model_.cy()));
  cv::Point3d v_right =
    pinhole_camera_model_.projectPixelTo3dRay(cv::Point2d(width, pinhole_camera_model_.cy()));

  double angle =
    std::abs(std::atan2(v_left.x, v_left.z)) + std::abs(std::atan2(v_right.x, v_right.z));

  // the calibration range is  hardcoded to 12m
  double total_area = CV_PI * 12.0 * 12.0 * angle / CV_2PI;

  double r = 0.5 * new_hypothesis_distance_;
  double current_area = converged_lidartag_hypotheses_.size() * CV_PI * r * r;

  return current_area / total_area;
}

int CalibrationEstimator::getCurrentCalibrationPairsNumber() const
{
  return converged_lidartag_hypotheses_.size();
}

double CalibrationEstimator::getCrossValidationReprojectionError() const
{
  return crossvalidation_reprojection_error_;
}

int CalibrationEstimator::getConvergencePairNumber() const { return convergence_min_pairs_; }
