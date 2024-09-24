// Copyright 2024 TIER IV, Inc.
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

#include <rclcpp/rclcpp.hpp>
#include <tier4_tag_utils/apriltag_hypothesis.hpp>

namespace tier4_tag_utils
{

ApriltagHypothesis::ApriltagHypothesis(
  int id, image_geometry::PinholeCameraModel & pinhole_camera_model)
: first_observation_(true), id_(id), pinhole_camera_model_(pinhole_camera_model)
{
}

ApriltagHypothesis::~ApriltagHypothesis() {}

bool ApriltagHypothesis::update(
  const std::vector<cv::Point2d> & corners, const rclcpp::Time & stamp)
{
  last_observation_timestamp_ = stamp;
  latest_corner_points_2d_ = corners;

  if (first_observation_) {
    first_observation_ = false;
    first_observation_timestamp_ = stamp;
    filtered_corner_points_2d_ = corners;

    initKalman(corners);
    return true;
  } else {
    cv::Point2d filtered_center = getCenter2d(filtered_corner_points_2d_);
    cv::Point2d current_center = getCenter2d(corners);

    if (cv::norm(filtered_center - current_center) > new_hypothesis_translation_) {
      first_observation_timestamp_ = stamp;
      filtered_corner_points_2d_ = corners;

      initKalman(corners);
      return false;
    }
  }

  for (int i = 0; i < 4; ++i) {
    cv::KalmanFilter & kalman_filter = kalman_filters_[i];

    cv::Mat prediction = kalman_filter.predict();
    cv::Mat observation = toState(corners[i]);

    cv::Mat estimated = kalman_filter.correct(observation);

    filtered_corner_points_2d_[i].x = estimated.at<double>(0);
    filtered_corner_points_2d_[i].y = estimated.at<double>(1);
  }

  return true;
}

bool ApriltagHypothesis::update(const rclcpp::Time & stamp)
{
  double since_last_observation = (stamp - last_observation_timestamp_).seconds();

  return since_last_observation < max_no_observation_time_;
}

int ApriltagHypothesis::getId() const { return id_; }

std::vector<cv::Point2d> ApriltagHypothesis::getLatestPoints2d() const
{
  return latest_corner_points_2d_;
}

std::vector<cv::Point2d> ApriltagHypothesis::getFilteredPoints2d() const
{
  return filtered_corner_points_2d_;
}

cv::Point2d ApriltagHypothesis::getCenter2d() const
{
  return getCenter2d(filtered_corner_points_2d_);
}

cv::Point2d ApriltagHypothesis::getCenter2d(const std::vector<cv::Point2d> & corners) const
{
  cv::Point2d center;
  assert(corners.size() == 4);

  for (int i = 0; i < 4; ++i) {
    center += corners[i];
  }

  center /= 4;

  return center;
}

std::vector<cv::Point3d> ApriltagHypothesis::getLatestPoints3d() const
{
  return getPoints3d(latest_corner_points_2d_);
}

std::vector<cv::Point3d> ApriltagHypothesis::getFilteredPoints3d() const
{
  return getPoints3d(filtered_corner_points_2d_);
}

std::vector<cv::Point3d> ApriltagHypothesis::getPoints3d(
  const std::vector<cv::Point2d> & image_points) const
{
  std::vector<cv::Point3d> object_points;

  std::vector<cv::Point3d> apriltag_template_points = {
    cv::Point3d(-0.5 * tag_size_, 0.5 * tag_size_, 0.0),
    cv::Point3d(0.5 * tag_size_, 0.5 * tag_size_, 0.0),
    cv::Point3d(0.5 * tag_size_, -0.5 * tag_size_, 0.0),
    cv::Point3d(-0.5 * tag_size_, -0.5 * tag_size_, 0.0)};

  cv::Mat rvec, tvec;

  bool success = cv::solvePnP(
    apriltag_template_points, image_points, pinhole_camera_model_.intrinsicMatrix(),
    pinhole_camera_model_.distortionCoeffs(), rvec, tvec, false, cv::SOLVEPNP_SQPNP);

  if (!success) {
    RCLCPP_ERROR(rclcpp::get_logger("tier4_tag_utils"), "PNP failed");
    return object_points;
  }

  cv::Matx31d translation_vector = tvec;
  cv::Matx33d rotation_matrix;

  translation_vector = tvec;
  cv::Rodrigues(rvec, rotation_matrix);

  for (cv::Point3d & template_point : apriltag_template_points) {
    cv::Matx31d p = rotation_matrix * cv::Matx31d(template_point) + translation_vector;
    object_points.push_back(cv::Point3d(p(0), p(1), p(2)));
  }

  return object_points;
}

cv::Point3d ApriltagHypothesis::getCenter3d() const { return getCenter3d(getFilteredPoints3d()); }

cv::Point3d ApriltagHypothesis::getCenter3d(const std::vector<cv::Point3d> & corners) const
{
  cv::Point3d center;
  assert(corners.size() == 4);

  for (int i = 0; i < 4; ++i) {
    center += corners[i];
  }

  center /= 4;

  return center;
}

bool ApriltagHypothesis::converged() const
{
  double seconds_since_first_observation =
    (last_observation_timestamp_ - first_observation_timestamp_).seconds();

  if (first_observation_ || seconds_since_first_observation < min_convergence_time_) {
    return false;
  }

  bool converged = true;

  for (int i = 0; i < 4; ++i) {
    const cv::KalmanFilter & kalman_filter = kalman_filters_[i];

    // decide based on the variance
    const cv::Mat & cov = kalman_filter.errorCovPost;

    double max_translation_cov = std::max({cov.at<double>(0, 0), cov.at<double>(1, 1)});

    if (std::sqrt(max_translation_cov) > convergence_translation_) {
      converged = false;

      break;
    }
  }

  return converged;
}

void ApriltagHypothesis::setMinConvergenceTime(double convergence_time)
{
  min_convergence_time_ = convergence_time;
}

void ApriltagHypothesis::setMaxConvergenceThreshold(double translation)
{
  convergence_translation_ = translation;
}

void ApriltagHypothesis::setNewHypothesisThreshold(double max_translation)
{
  new_hypothesis_translation_ = max_translation;
}

void ApriltagHypothesis::setMaxNoObservationTime(double time) { max_no_observation_time_ = time; }

void ApriltagHypothesis::setMeasurementNoise(double translation)
{
  measurement_noise_translation_ = translation;
}

void ApriltagHypothesis::setProcessNoise(double translation)
{
  process_noise_translation_ = translation;
}

void ApriltagHypothesis::setTagSize(double size) { tag_size_ = size; }

void ApriltagHypothesis::initKalman(const std::vector<cv::Point2d> & corners)
{
  for (int i = 0; i < 4; ++i) {
    cv::KalmanFilter & kalman_filter = kalman_filters_[i];
    kalman_filter.init(2, 2, 0, CV_64F);  // states x observations

    const double process_cov_translation = process_noise_translation_ * process_noise_translation_;

    cv::setIdentity(kalman_filter.processNoiseCov, cv::Scalar::all(1.0));

    kalman_filter.processNoiseCov.at<double>(0, 0) = process_cov_translation;
    kalman_filter.processNoiseCov.at<double>(1, 1) = process_cov_translation;

    const double measurement_cov_translation =
      measurement_noise_translation_ * measurement_noise_translation_;

    cv::setIdentity(kalman_filter.measurementNoiseCov, cv::Scalar::all(1.0));

    kalman_filter.measurementNoiseCov.at<double>(0, 0) = measurement_cov_translation;
    kalman_filter.measurementNoiseCov.at<double>(1, 1) = measurement_cov_translation;

    cv::setIdentity(kalman_filter.errorCovPost, cv::Scalar::all(1.0));
    cv::setIdentity(kalman_filter.transitionMatrix, cv::Scalar::all(1.0));
    cv::setIdentity(kalman_filter.measurementMatrix, cv::Scalar::all(1.0));

    kalman_filter.statePost = toState(corners[i]);
  }
}

cv::Mat ApriltagHypothesis::toState(const cv::Point2d & corner)
{
  cv::Mat kalman_state(2, 1, CV_64F);

  kalman_state.at<double>(0, 0) = corner.x;
  kalman_state.at<double>(1, 0) = corner.y;

  return kalman_state;
}

}  // namespace tier4_tag_utils
