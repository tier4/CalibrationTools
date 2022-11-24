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

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <tier4_tag_utils/lidartag_hypothesis.hpp>

namespace tier4_tag_utils
{

LidartagHypothesis::LidartagHypothesis(int id)
: id_(id), first_observation_(true), estimated_speed_(0.0)
{
}

bool LidartagHypothesis::update(
  const cv::Matx31d & pose_translation, const cv::Matx33d & pose_rotation, double tag_size,
  const rclcpp::Time & stamp)
{
  tag_size_ = tag_size;
  double dt = first_observation_ ? 0.0 : (stamp - last_observation_timestamp_).seconds();
  last_observation_timestamp_ = stamp;
  latest_translation_vector_ = pose_translation;
  latest_rotation_matrix_ = pose_rotation;

  double trans_diff = cv::norm(filtered_translation_vector_ - pose_translation);
  double ang_diff = std::acos(std::min(
    1.0, 0.5 * (cv::trace(filtered_rotation_matrix_.inv() * pose_rotation) -
                1.0)));  // Tr(R) = 1 + 2*cos(theta)

  if (first_observation_) {
    first_observation_ = false;
    first_observation_timestamp_ = stamp;
    filtered_translation_vector_ = pose_translation;
    filtered_rotation_matrix_ = pose_rotation;

    initKalman(pose_translation, pose_rotation);
    return true;
  } else if (
    trans_diff > new_hypothesis_transl_ || ang_diff > new_hypothesis_rot_ ||
    dt > max_no_observation_time_) {
    first_observation_timestamp_ = stamp;
    filtered_translation_vector_ = pose_translation;
    filtered_rotation_matrix_ = pose_rotation;

    initKalman(pose_translation, pose_rotation);

    cv::Matx31d eulers_estimated;
    cv::Mat & statePost = kalman_filter_.statePost;
    eulers_estimated(0) = statePost.at<double>(3);
    eulers_estimated(1) = statePost.at<double>(4);
    eulers_estimated(2) = statePost.at<double>(5);

    return false;
  }

  if (dynamics_model_ == DynamicsModel::Static) {
    cv::Mat prediction = kalman_filter_.predict();
    cv::Mat observation = toState(pose_translation, pose_rotation);
    fixState(prediction, observation);
    cv::Mat estimated = kalman_filter_.correct(observation);
    fixState(kalman_filter_.statePost);

    filtered_translation_vector_(0) = estimated.at<double>(0);
    filtered_translation_vector_(1) = estimated.at<double>(1);
    filtered_translation_vector_(2) = estimated.at<double>(2);

    cv::Matx31d eulers_estimated;
    eulers_estimated(0) = estimated.at<double>(3);
    eulers_estimated(1) = estimated.at<double>(4);
    eulers_estimated(2) = estimated.at<double>(5);

    filtered_rotation_matrix_ = euler2rot(eulers_estimated);

    double current_speed = cv::norm(pose_translation - filtered_translation_vector_) / dt;
    estimated_speed_ = dt > 0.0 ? 0.8 * estimated_speed_ + 0.2 * current_speed : 0.0;

  } else {
    // non-fixed timestep
    kalman_filter_.transitionMatrix.at<double>(0, 3) = dt;
    kalman_filter_.transitionMatrix.at<double>(1, 4) = dt;
    kalman_filter_.transitionMatrix.at<double>(2, 5) = dt;
    kalman_filter_.transitionMatrix.at<double>(6, 9) = dt;
    kalman_filter_.transitionMatrix.at<double>(7, 10) = dt;
    kalman_filter_.transitionMatrix.at<double>(8, 11) = dt;

    cv::Mat prediction = kalman_filter_.predict();
    cv::Mat observation = toObservation(pose_translation, pose_rotation);
    fixState(prediction, observation);
    cv::Mat estimated = kalman_filter_.correct(observation);
    fixState(kalman_filter_.statePost);

    filtered_translation_vector_(0) = estimated.at<double>(0);
    filtered_translation_vector_(1) = estimated.at<double>(1);
    filtered_translation_vector_(2) = estimated.at<double>(2);

    cv::Matx31d eulers_estimated;
    eulers_estimated(0) = estimated.at<double>(6);
    eulers_estimated(1) = estimated.at<double>(7);
    eulers_estimated(2) = estimated.at<double>(8);

    filtered_rotation_matrix_ = euler2rot(eulers_estimated);
  }

  return true;
}

bool LidartagHypothesis::update(const rclcpp::Time & stamp)
{
  double since_last_observation = (stamp - last_observation_timestamp_).seconds();
  return since_last_observation < max_no_observation_time_;
}

int LidartagHypothesis::getId() const { return id_; }

std::vector<cv::Point3d> LidartagHypothesis::getLatestPoints()
{
  std::vector<cv::Point2d> vertex = {
    cv::Point2d(-0.75f, -0.75f), cv::Point2d(0.75f, -0.75f), cv::Point2d(0.75f, 0.75f),
    cv::Point2d(-0.75f, 0.75f)};

  std::vector<cv::Point3d> corners;
  corners.resize(4);

  for (int i = 0; i < 4; ++i) {
    const cv::Point2d & v = vertex[i];
    cv::Matx31d corner(0.f, v.x * tag_size_ / 2.f, v.y * tag_size_ / 2.f);
    corner = latest_rotation_matrix_ * corner + latest_translation_vector_;

    corners[i].x = corner(0, 0);
    corners[i].y = corner(1, 0);
    corners[i].z = corner(2, 0);
  }

  latest_corner_points_ = corners;

  return corners;
}

cv::Matx33d LidartagHypothesis::getLatestRotation() const { return latest_rotation_matrix_; }

cv::Matx31d LidartagHypothesis::getLatestTranslation() const { return latest_translation_vector_; }

std::vector<cv::Point3d> LidartagHypothesis::getFilteredPoints()
{
  std::vector<cv::Point2d> vertex = {
    cv::Point2d(-0.75f, -0.75f), cv::Point2d(0.75f, -0.75f), cv::Point2d(0.75f, 0.75f),
    cv::Point2d(-0.75f, 0.75f)};

  std::vector<cv::Point3d> corners;
  corners.resize(4);

  for (int i = 0; i < 4; ++i) {
    const cv::Point2d & v = vertex[i];
    cv::Matx31d corner(0.f, v.x * tag_size_ / 2.f, v.y * tag_size_ / 2.f);
    corner = filtered_rotation_matrix_ * corner + filtered_translation_vector_;

    corners[i].x = corner(0, 0);
    corners[i].y = corner(1, 0);
    corners[i].z = corner(2, 0);
  }

  filtered_corner_points = corners;

  return corners;
}

cv::Matx33d LidartagHypothesis::getFilteredRotation() const { return filtered_rotation_matrix_; }

cv::Matx31d LidartagHypothesis::getFilteredTranslation() const
{
  return filtered_translation_vector_;
}

cv::Point3d LidartagHypothesis::getCenter() const
{
  return cv::Point3d(
    filtered_translation_vector_(0), filtered_translation_vector_(1),
    filtered_translation_vector_(2));
}

double LidartagHypothesis::getTransCov() const
{
  const cv::Mat & cov = kalman_filter_.errorCovPost;

  double max_transl_cov =
    std::max({cov.at<double>(0, 0), cov.at<double>(1, 1), cov.at<double>(2, 2)});

  return std::sqrt(max_transl_cov);
}

double LidartagHypothesis::getTransDotCov() const
{
  const cv::Mat & cov = kalman_filter_.errorCovPost;

  double max_transl_dot_cov =
    dynamics_model_ == DynamicsModel::Static
      ? 0.0
      : std::max({cov.at<double>(3, 3), cov.at<double>(4, 4), cov.at<double>(5, 5)});

  return std::sqrt(max_transl_dot_cov);
}

double LidartagHypothesis::getRotCov() const
{
  const cv::Mat & cov = kalman_filter_.errorCovPost;

  double max_rot_cov =
    dynamics_model_ == DynamicsModel::Static
      ? std::max({cov.at<double>(3, 3), cov.at<double>(4, 4), cov.at<double>(5, 5)})
      : std::max({cov.at<double>(6, 6), cov.at<double>(7, 7), cov.at<double>(8, 8)});

  return std::sqrt(max_rot_cov);
}

double LidartagHypothesis::getRotDotCov() const
{
  const cv::Mat & cov = kalman_filter_.errorCovPost;

  double max_rot_dot_cov =
    dynamics_model_ == DynamicsModel::Static
      ? 0.0
      : std::max({cov.at<double>(9, 9), cov.at<double>(10, 10), cov.at<double>(11, 11)});

  return std::sqrt(max_rot_dot_cov);
}

double LidartagHypothesis::getSpeed() const
{
  const cv::Mat & state = kalman_filter_.statePost;

  if (dynamics_model_ == DynamicsModel::Static) {
    return estimated_speed_;
  }

  double vx = state.at<double>(3);
  double vy = state.at<double>(3);
  double vz = state.at<double>(3);

  return std::sqrt(vx * vx + vy * vy + vz * vz);
}

bool LidartagHypothesis::converged() const
{
  double seconds_since_first_observation =
    (last_observation_timestamp_ - first_observation_timestamp_).seconds();

  if (first_observation_ || seconds_since_first_observation < min_convergence_time_) {
    return false;
  }

  // decide based on the variance
  const cv::Mat & cov = kalman_filter_.errorCovPost;

  double max_transl_cov =
    std::max({cov.at<double>(0, 0), cov.at<double>(1, 1), cov.at<double>(2, 2)});

  double max_transl_dot_cov =
    dynamics_model_ == DynamicsModel::Static
      ? 0.0
      : std::max({cov.at<double>(3, 3), cov.at<double>(4, 4), cov.at<double>(5, 5)});

  double max_rot_cov =
    dynamics_model_ == DynamicsModel::Static
      ? std::max({cov.at<double>(3, 3), cov.at<double>(4, 4), cov.at<double>(5, 5)})
      : std::max({cov.at<double>(6, 6), cov.at<double>(7, 7), cov.at<double>(8, 8)});

  if (
    std::sqrt(max_transl_cov) > convergence_transl_ ||
    std::sqrt(max_transl_dot_cov) > convergence_transl_dot_ ||
    std::sqrt(max_rot_cov) > convergence_rot_ ||
    std::sqrt(max_transl_dot_cov) > convergence_rot_dot_ || getSpeed() > convergence_transl_dot_) {
    return false;
  }

  return true;
}

double LidartagHypothesis::timeSinceFirstObservation(const rclcpp::Time & stamp) const
{
  return (stamp - first_observation_timestamp_).seconds();
}

double LidartagHypothesis::timeSinceLastObservation(const rclcpp::Time & stamp) const
{
  return (stamp - last_observation_timestamp_).seconds();
}

void LidartagHypothesis::setDynamicsModel(DynamicsModel dynamics_model)
{
  dynamics_model_ = dynamics_model;
}

void LidartagHypothesis::setMinConvergenceTime(double convergence_time)
{
  min_convergence_time_ = convergence_time;
}

void LidartagHypothesis::setMaxConvergenceThreshold(
  double transl, double transl_dot, double rot, double rot_dot)
{
  convergence_transl_ = transl;
  convergence_transl_dot_ = transl_dot;
  convergence_rot_ = rot;
  convergence_rot_dot_ = rot_dot;
}

void LidartagHypothesis::setNewHypothesisThreshold(double max_transl, double max_rot)
{
  new_hypothesis_transl_ = max_transl;
  new_hypothesis_rot_ = max_rot;
}

void LidartagHypothesis::setMaxNoObservationTime(double time) { max_no_observation_time_ = time; }

void LidartagHypothesis::setMeasurementNoise(double transl, double rot)
{
  measurement_noise_transl_ = transl;
  measurement_noise_rot_ = rot;
}

void LidartagHypothesis::setProcessNoise(
  double transl, double transl_dot, double rot, double rot_dot)
{
  process_noise_transl_ = transl;
  process_noise_transl_dot_ = transl_dot;
  process_noise_rot_ = rot;
  process_noise_rot_dot_ = rot_dot;
}

void LidartagHypothesis::initKalman(
  const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix)
{
  if (dynamics_model_ == DynamicsModel::Static) {
    initStaticKalman(translation_vector, rotation_matrix);
  } else {
    initConstantVelocityKalman(translation_vector, rotation_matrix);
  }
}

void LidartagHypothesis::initStaticKalman(
  const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix)
{
  kalman_filter_.init(6, 6, 0, CV_64F);

  const double process_cov_transl = process_noise_transl_ * process_noise_transl_;
  const double process_cov_rot = process_noise_rot_ * process_noise_rot_;

  cv::setIdentity(kalman_filter_.processNoiseCov, cv::Scalar::all(1.0));

  kalman_filter_.processNoiseCov.at<double>(0, 0) = process_cov_transl;
  kalman_filter_.processNoiseCov.at<double>(1, 1) = process_cov_transl;
  kalman_filter_.processNoiseCov.at<double>(2, 2) = process_cov_transl;
  kalman_filter_.processNoiseCov.at<double>(3, 3) = process_cov_rot;
  kalman_filter_.processNoiseCov.at<double>(4, 4) = process_cov_rot;
  kalman_filter_.processNoiseCov.at<double>(5, 5) = process_cov_rot;

  const double measurement_cov_transl = measurement_noise_transl_ * measurement_noise_transl_;
  const double measurement_cov_rot = measurement_noise_rot_ * measurement_noise_rot_;

  cv::setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar::all(1.0));

  kalman_filter_.measurementNoiseCov.at<double>(0, 0) = measurement_cov_transl;
  kalman_filter_.measurementNoiseCov.at<double>(1, 1) = measurement_cov_transl;
  kalman_filter_.measurementNoiseCov.at<double>(2, 2) = measurement_cov_transl;
  kalman_filter_.measurementNoiseCov.at<double>(3, 3) = measurement_cov_rot;
  kalman_filter_.measurementNoiseCov.at<double>(4, 4) = measurement_cov_rot;
  kalman_filter_.measurementNoiseCov.at<double>(5, 5) = measurement_cov_rot;

  cv::setIdentity(kalman_filter_.errorCovPost, cv::Scalar::all(1.0));
  cv::setIdentity(kalman_filter_.transitionMatrix, cv::Scalar::all(1.0));
  cv::setIdentity(kalman_filter_.measurementMatrix, cv::Scalar::all(1.0));

  kalman_filter_.statePost = toState(translation_vector, rotation_matrix);
}

void LidartagHypothesis::initConstantVelocityKalman(
  const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix)
{
  kalman_filter_.init(12, 6, 0, CV_64F);

  double dt = 1.0;
  const double process_cov_transl = process_noise_transl_ * process_noise_transl_;
  const double process_cov_transl_dot = process_noise_transl_dot_ * process_noise_transl_dot_;
  const double process_cov_rot = process_noise_rot_ * process_noise_rot_;
  const double process_cov_rot_dot = process_noise_rot_dot_ * process_noise_rot_dot_;

  cv::setIdentity(kalman_filter_.processNoiseCov, cv::Scalar::all(1.0));

  kalman_filter_.processNoiseCov.at<double>(0, 0) = process_cov_transl;
  kalman_filter_.processNoiseCov.at<double>(1, 1) = process_cov_transl;
  kalman_filter_.processNoiseCov.at<double>(2, 2) = process_cov_transl;
  kalman_filter_.processNoiseCov.at<double>(3, 3) = process_cov_transl_dot;
  kalman_filter_.processNoiseCov.at<double>(4, 4) = process_cov_transl_dot;
  kalman_filter_.processNoiseCov.at<double>(5, 5) = process_cov_transl_dot;
  kalman_filter_.processNoiseCov.at<double>(6, 6) = process_cov_rot;
  kalman_filter_.processNoiseCov.at<double>(7, 7) = process_cov_rot;
  kalman_filter_.processNoiseCov.at<double>(8, 8) = process_cov_rot;
  kalman_filter_.processNoiseCov.at<double>(9, 9) = process_cov_rot_dot;
  kalman_filter_.processNoiseCov.at<double>(10, 10) = process_cov_rot_dot;
  kalman_filter_.processNoiseCov.at<double>(11, 11) = process_cov_rot_dot;

  const double measurement_cov_transl = measurement_noise_transl_ * measurement_noise_transl_;
  const double measurement_cov_rot = measurement_noise_rot_ * measurement_noise_rot_;

  cv::setIdentity(kalman_filter_.measurementNoiseCov, cv::Scalar::all(1.0));

  kalman_filter_.measurementNoiseCov.at<double>(0, 0) = measurement_cov_transl;
  kalman_filter_.measurementNoiseCov.at<double>(1, 1) = measurement_cov_transl;
  kalman_filter_.measurementNoiseCov.at<double>(2, 2) = measurement_cov_transl;
  kalman_filter_.measurementNoiseCov.at<double>(3, 3) = measurement_cov_rot;
  kalman_filter_.measurementNoiseCov.at<double>(4, 4) = measurement_cov_rot;
  kalman_filter_.measurementNoiseCov.at<double>(5, 5) = measurement_cov_rot;

  cv::setIdentity(kalman_filter_.errorCovPost, cv::Scalar::all(1.0));

  /* Constant velocity model */
  //  [1 0 0 dt  0  0  0 0 0  0  0  0 ]
  //  [0 1 0  0 dt  0  0 0 0  0  0  0 ]
  //  [0 0 1  0  0 dt  0 0 0  0  0  0 ]
  //  [0 0 0  1  0  0  0 0 0  0  0  0 ]
  //  [0 0 0  0  1  0  0 0 0  0  0  0 ]
  //  [0 0 0  0  0  1  0 0 0  0  0  0 ]
  //  [0 0 0  0  0  0  1 0 0 dt  0  0 ]
  //  [0 0 0  0  0  0  0 1 0  0 dt  0 ]
  //  [0 0 0  0  0  0  0 0 1  0  0 dt ]
  //  [0 0 0  0  0  0  0 0 0  1  0  0 ]
  //  [0 0 0  0  0  0  0 0 0  0  1  0 ]
  //  [0 0 0  0  0  0  0 0 0  0  0  1 ]
  cv::setIdentity(kalman_filter_.transitionMatrix, cv::Scalar::all(1.0));

  // position
  kalman_filter_.transitionMatrix.at<double>(0, 3) = dt;
  kalman_filter_.transitionMatrix.at<double>(1, 4) = dt;
  kalman_filter_.transitionMatrix.at<double>(2, 5) = dt;

  // orientation
  kalman_filter_.transitionMatrix.at<double>(6, 9) = dt;
  kalman_filter_.transitionMatrix.at<double>(7, 10) = dt;
  kalman_filter_.transitionMatrix.at<double>(8, 11) = dt;

  /* MEASUREMENT MODEL */
  //  [1 0 0 0 0 0 0 0 0 0 0 0]
  //  [0 1 0 0 0 0 0 0 0 0 0 0]
  //  [0 0 1 0 0 0 0 0 0 0 0 0]
  //  [0 0 0 0 0 0 1 0 0 0 0 0]
  //  [0 0 0 0 0 0 0 1 0 0 0 0]
  //  [0 0 0 0 0 0 0 0 1 0 0 0]
  cv::setIdentity(kalman_filter_.measurementMatrix, cv::Scalar::all(0.0));

  kalman_filter_.measurementMatrix.at<double>(0, 0) = 1.0;  // x
  kalman_filter_.measurementMatrix.at<double>(1, 1) = 1.0;  // y
  kalman_filter_.measurementMatrix.at<double>(2, 2) = 1.0;  // z
  kalman_filter_.measurementMatrix.at<double>(3, 6) = 1.0;  // roll
  kalman_filter_.measurementMatrix.at<double>(4, 7) = 1.0;  // pitch
  kalman_filter_.measurementMatrix.at<double>(5, 8) = 1.0;  // yaw

  kalman_filter_.statePost = toState(translation_vector, rotation_matrix);
}

cv::Mat LidartagHypothesis::toState(
  const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix)
{
  cv::Matx31d euler_angles = rot2euler(rotation_matrix);

  if (dynamics_model_ == DynamicsModel::Static) {
    cv::Mat kalman_state(6, 1, CV_64F);

    kalman_state.at<double>(0, 0) = translation_vector(0, 0);
    kalman_state.at<double>(1, 0) = translation_vector(1, 0);
    kalman_state.at<double>(2, 0) = translation_vector(2, 0);
    kalman_state.at<double>(3, 0) = euler_angles(0, 0);
    kalman_state.at<double>(4, 0) = euler_angles(1, 0);
    kalman_state.at<double>(5, 0) = euler_angles(2, 0);

    return kalman_state;
  } else {
    cv::Mat kalman_state(12, 1, CV_64F);
    kalman_state.setTo(cv::Scalar(0.0));

    kalman_state.at<double>(0, 0) = translation_vector(0, 0);
    kalman_state.at<double>(1, 0) = translation_vector(1, 0);
    kalman_state.at<double>(2, 0) = translation_vector(2, 0);
    kalman_state.at<double>(6, 0) = euler_angles(0, 0);
    kalman_state.at<double>(7, 0) = euler_angles(1, 0);
    kalman_state.at<double>(8, 0) = euler_angles(2, 0);

    return kalman_state;
  }
}

cv::Mat LidartagHypothesis::toObservation(
  const cv::Matx31d & translation_vector, const cv::Matx33d & rotation_matrix)
{
  cv::Matx31d euler_angles = rot2euler(rotation_matrix);

  cv::Mat kalman_state(6, 1, CV_64F);

  kalman_state.at<double>(0, 0) = translation_vector(0, 0);
  kalman_state.at<double>(1, 0) = translation_vector(1, 0);
  kalman_state.at<double>(2, 0) = translation_vector(2, 0);
  kalman_state.at<double>(3, 0) = euler_angles(0, 0);
  kalman_state.at<double>(4, 0) = euler_angles(1, 0);
  kalman_state.at<double>(5, 0) = euler_angles(2, 0);

  return kalman_state;
}

void LidartagHypothesis::fixState(cv::Mat & old_state, cv::Mat & new_prediction)
{
  if (dynamics_model_ == DynamicsModel::Static) {
    cv::Mat kalman_state(6, 1, CV_64F);

    double old_x = old_state.at<double>(3, 0);
    double old_z = old_state.at<double>(5, 0);

    double & new_x = new_prediction.at<double>(3, 0);
    double & new_z = new_prediction.at<double>(5, 0);

    new_x = std::abs(new_x + CV_2PI - old_x) < std::abs(new_x - old_x)   ? new_x + CV_2PI
            : std::abs(new_x - CV_2PI - old_x) < std::abs(new_x - old_x) ? new_x - CV_2PI
                                                                         : new_x;

    new_z = std::abs(new_z + CV_2PI - old_z) < std::abs(new_z - old_z)   ? new_z + CV_2PI
            : std::abs(new_z - CV_2PI - old_z) < std::abs(new_z - old_z) ? new_z - CV_2PI
                                                                         : new_z;

    return;
  } else {
    double old_x = old_state.at<double>(6, 0);
    double old_z = old_state.at<double>(8, 0);

    double & new_x = new_prediction.at<double>(3, 0);
    double & new_z = new_prediction.at<double>(5, 0);

    new_x = std::abs(new_x + CV_2PI - old_x) < std::abs(new_x - old_x)   ? new_x + CV_2PI
            : std::abs(new_x - CV_2PI - old_x) < std::abs(new_x - old_x) ? new_x - CV_2PI
                                                                         : new_x;

    new_z = std::abs(new_z + CV_2PI - old_z) < std::abs(new_z - old_z)   ? new_z + CV_2PI
            : std::abs(new_z - CV_2PI - old_z) < std::abs(new_z - old_z) ? new_z - CV_2PI
                                                                         : new_z;

    return;
  }
}

void LidartagHypothesis::fixState(cv::Mat & new_state)
{
  if (dynamics_model_ == DynamicsModel::Static) {
    double & new_x = new_state.at<double>(3, 0);
    double & new_z = new_state.at<double>(5, 0);

    new_x = std::min(std::max(-CV_PI, new_x), CV_PI);
    new_z = std::min(std::max(-CV_PI, new_z), CV_PI);
    return;
  } else {
    double & new_x = new_state.at<double>(6, 0);
    double & new_z = new_state.at<double>(8, 0);

    new_x = std::min(std::max(-CV_PI, new_x), CV_PI);
    new_z = std::min(std::max(-CV_PI, new_z), CV_PI);
    return;
  }
}

// Converts a given Rotation Matrix to Euler angles
cv::Matx31d LidartagHypothesis::rot2euler(const cv::Matx33d & rotation_matrix)
{
  cv::Mat rotation_matrix_cv(rotation_matrix);
  Eigen::Matrix3d rotation_matrix_eigen;
  cv::cv2eigen(rotation_matrix_cv, rotation_matrix_eigen);

  Eigen::Vector3d euler = rotation_matrix_eigen.eulerAngles(0, 1, 2);

  return cv::Matx31d(euler.x(), euler.y(), euler.z());
  ;
}

// Converts a given Euler angles to Rotation Matrix
cv::Matx33d LidartagHypothesis::euler2rot(const cv::Matx31d & euler)
{
  double x = euler(0);
  double y = euler(1);
  double z = euler(2);

  Eigen::Matrix3d rotation_matrix_eigen =
    Eigen::AngleAxisd(x, Eigen::Vector3d::UnitX()) *
    Eigen::AngleAxisd(y, Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(z, Eigen::Vector3d::UnitZ()).toRotationMatrix();

  cv::Mat rotation_matrix_cv;
  cv::eigen2cv(rotation_matrix_eigen, rotation_matrix_cv);

  return rotation_matrix_cv;
}

}  // namespace tier4_tag_utils
