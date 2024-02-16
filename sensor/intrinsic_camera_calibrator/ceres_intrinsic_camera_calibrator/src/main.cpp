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

// #include <extrinsic_tag_based_base_calibrator/extrinsic_tag_based_base_calibrator.hpp>
// #include <rclcpp/rclcpp.hpp>

#include <ceres_intrinsic_camera_calibrator/ceres_camera_intrinsics_optimizer.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char ** argv)
{
  CV_UNUSED(argc);
  CV_UNUSED(argv);

  // Global config
  int cols = 6;
  int rows = 8;
  std::size_t max_samples = 50;
  std::size_t mini_calibration_samples = 20;
  bool use_tangent_distortion = true;
  int num_radial_distortion_coeffs = 3;

  // Placeholders
  std::vector<std::vector<cv::Point3f>> all_object_points;
  std::vector<std::vector<cv::Point2f>> all_image_points;

  std::vector<std::vector<cv::Point3f>> mini_calibration_object_points;
  std::vector<std::vector<cv::Point2f>> mini_calibration_image_points;
  std::vector<std::vector<cv::Point3f>> calibration_object_points;
  std::vector<std::vector<cv::Point2f>> calibration_image_points;
  std::vector<cv::Mat> mini_opencv_calibration_rvecs;
  std::vector<cv::Mat> mini_opencv_calibration_tvecs;
  std::vector<cv::Mat> opencv_calibration_rvecs;
  std::vector<cv::Mat> opencv_calibration_tvecs;
  std::vector<cv::Mat> ceres_calibration_rvecs;
  std::vector<cv::Mat> ceres_calibration_tvecs;

  std::string data_path = std::string(argv[1]);
  std::vector<std::string> image_paths;

  // write code to iterate through a folder
  const std::filesystem::path fs_data_path{data_path};

  for (const auto & entry : std::filesystem::directory_iterator(fs_data_path)) {
    const auto file_name = entry.path().string();
    if (entry.is_regular_file()) {
      image_paths.push_back(file_name);
    }
  }

  std::cout << "Image files: " << image_paths.size() << std::endl;

  double min_area_percentage = 0.01;
  double max_area_percentage = 1.2;
  double min_dist_between_blobs_percentage = 1.0;

  cv::SimpleBlobDetector::Params params;
  params.filterByArea = true;

  cv::Ptr<cv::FeatureDetector> blobDetector = cv::SimpleBlobDetector::create(params);

  std::vector<cv::Point3f> template_points;
  for (int j = 0; j < rows; j++) {
    for (int i = 0; i < cols; i++) {
      template_points.emplace_back(i, j, 0.0);
    }
  }

  cv::Size size(-1, -1);

  for (std::size_t i = 0; i < image_paths.size(); ++i) {
    cv::Mat grayscale_img =
      cv::imread(image_paths[i], cv::IMREAD_GRAYSCALE | cv::IMREAD_IGNORE_ORIENTATION);

    assert(size.height == -1 || size.height == grayscale_img.rows);
    assert(size.width == -1 || size.width == grayscale_img.cols);
    size = grayscale_img.size();

    params.minArea = min_area_percentage * size.height * size.width / 100.0;
    params.maxArea = max_area_percentage * size.height * size.width / 100.0;
    params.minDistBetweenBlobs =
      (min_dist_between_blobs_percentage * std::max(size.height, size.width) / 100.0);

    cv::Size pattern(cols, rows);      // w x h format
    std::vector<cv::Point2f> centers;  // this will be filled by the detected centers

    bool found = cv::findCirclesGrid(
      grayscale_img, pattern, centers, cv::CALIB_CB_SYMMETRIC_GRID | cv::CALIB_CB_CLUSTERING,
      blobDetector);

    if (found) {
      all_object_points.push_back(template_points);
      all_image_points.push_back(centers);
    }

    if (all_object_points.size() >= max_samples) {
      break;
    }

    std::cout << "|" << std::flush;
  }

  std::cout << "Board detections: " << all_object_points.size() << std::endl;

  // Fill the calibration points
  calibration_object_points.insert(
    calibration_object_points.end(), all_object_points.begin(),
    all_object_points.begin() + max_samples);
  calibration_image_points.insert(
    calibration_image_points.end(), all_image_points.begin(),
    all_image_points.begin() + max_samples);

  mini_calibration_object_points.insert(
    mini_calibration_object_points.end(), all_object_points.begin(),
    all_object_points.begin() + mini_calibration_samples);
  mini_calibration_image_points.insert(
    mini_calibration_image_points.end(), all_image_points.begin(),
    all_image_points.begin() + mini_calibration_samples);

  cv::Mat_<double> mini_opencv_camera_matrix = cv::Mat_<double>::zeros(3, 3);
  cv::Mat_<double> mini_opencv_dist_coeffs = cv::Mat_<double>::zeros(5, 1);

  cv::Mat_<double> opencv_camera_matrix = cv::Mat_<double>::zeros(3, 3);
  cv::Mat_<double> opencv_dist_coeffs = cv::Mat_<double>::zeros(5, 1);
  cv::Mat_<double> undistorted_camera_matrix = cv::Mat_<double>::zeros(3, 3);

  cv::Mat_<double> ceres_camera_matrix = cv::Mat_<double>::zeros(3, 3);
  cv::Mat_<double> ceres_dist_coeffs = cv::Mat_<double>::zeros(5, 1);

  int flags = 0;

  if (!use_tangent_distortion) {
    flags |= cv::CALIB_ZERO_TANGENT_DIST;
  }

  if (num_radial_distortion_coeffs < 3) {
    flags |= cv::CALIB_FIX_K3;
  }

  if (num_radial_distortion_coeffs < 2) {
    flags |= cv::CALIB_FIX_K2;
  }

  if (num_radial_distortion_coeffs < 1) {
    flags |= cv::CALIB_FIX_K1;
  }

  auto mini_opencv_start = std::chrono::high_resolution_clock::now();
  double mini_reproj_error = cv::calibrateCamera(
    mini_calibration_object_points, mini_calibration_image_points, size, mini_opencv_camera_matrix,
    mini_opencv_dist_coeffs, mini_opencv_calibration_rvecs, mini_opencv_calibration_tvecs, flags);

  auto mini_opencv_stop = std::chrono::high_resolution_clock::now();

  std::cout << "Mini opencv calibration results" << std::endl;
  std::cout << "\tcamera_matrix: \n" << mini_opencv_camera_matrix << std::endl;
  std::cout << "\tdist_coeffs: \n" << mini_opencv_dist_coeffs << std::endl;
  std::cout << "\tmini_opencv_calibration_rvecs[0]: \n"
            << mini_opencv_calibration_rvecs[0] << std::endl;
  std::cout << "\tmini_opencv_calibration_rvecs[0]: \n"
            << mini_opencv_calibration_tvecs[0] << std::endl;

  auto opencv_start = std::chrono::high_resolution_clock::now();

  double reproj_error = cv::calibrateCamera(
    calibration_object_points, calibration_image_points, size, opencv_camera_matrix,
    opencv_dist_coeffs, opencv_calibration_rvecs, opencv_calibration_tvecs, flags);

  auto opencv_stop = std::chrono::high_resolution_clock::now();

  std::cout << "Opencv calibration results" << std::endl;
  std::cout << "\tcamera_matrix: \n" << opencv_camera_matrix << std::endl;
  std::cout << "\tdist_coeffs: \n" << opencv_dist_coeffs << std::endl;

  std::cout << "Mini OpenCV calibration error (reported by the calibrator)=" << mini_reproj_error
            << std::endl;
  std::cout << "OpenCV calibration error (reported by the calibrator)=" << reproj_error
            << std::endl;

  // Need to compute the whole rvecs, tvecs for the whole calibration set
  auto ceres_start = std::chrono::high_resolution_clock::now();

  for (std::size_t i = mini_calibration_object_points.size(); i < calibration_object_points.size();
       i++) {
    std::vector<cv::Point2f> calibration_projected_points;
    std::vector<cv::Point2f> pnp_projected_points;

    cv::Mat rvec, tvec;
    bool status = cv::solvePnP(
      calibration_object_points[i], calibration_image_points[i], mini_opencv_camera_matrix,
      mini_opencv_dist_coeffs, rvec, tvec);
    CV_UNUSED(status);
    mini_opencv_calibration_rvecs.push_back(rvec);
    mini_opencv_calibration_tvecs.push_back(tvec);
  }

  CeresCameraIntrinsicsOptimizer optimizer;
  optimizer.setRadialDistortionCoefficients(num_radial_distortion_coeffs);
  optimizer.setTangentialDistortion(use_tangent_distortion);
  optimizer.setVerbose(true);
  optimizer.setData(
    mini_opencv_camera_matrix, mini_opencv_dist_coeffs, calibration_object_points,
    calibration_image_points, mini_opencv_calibration_rvecs, mini_opencv_calibration_tvecs);
  optimizer.dataToPlaceholders();
  optimizer.evaluate();
  optimizer.solve();
  optimizer.placeholdersToData();
  optimizer.evaluate();
  double rms_rror = optimizer.getSolution(
    ceres_camera_matrix, ceres_dist_coeffs, ceres_calibration_rvecs, ceres_calibration_tvecs);
  (void)rms_rror;

  auto ceres_stop = std::chrono::high_resolution_clock::now();

  std::cout << "Ceres calibration results" << std::endl;
  std::cout << "\tcamera_matrix: \n" << ceres_camera_matrix << std::endl;
  std::cout << "\tdist_coeffs: \n" << ceres_dist_coeffs << std::endl;

  std::cout << "Mini opencv calibration results" << std::endl;
  std::cout << "\tcamera_matrix: \n" << mini_opencv_camera_matrix << std::endl;
  std::cout << "\tdist_coeffs: \n" << mini_opencv_dist_coeffs << std::endl;
  std::cout << "\tmini_opencv_calibration_rvecs[0]: \n"
            << mini_opencv_calibration_rvecs[0] << std::endl;
  std::cout << "\tmini_opencv_calibration_rvecs[0]: \n"
            << mini_opencv_calibration_tvecs[0] << std::endl;

  // Start developing the ceres optimizer
  double total_mini_opencv_calibration_error = 0.0;
  double total_opencv_calibration_error = 0.0;
  double total_ceres_calibration_error = 0.0;
  auto get_reprojection_error = [](auto & image_points, auto & projected_points) -> double {
    cv::Mat x = cv::Mat(2 * image_points.size(), 1, CV_32F, image_points.data());
    cv::Mat y = cv::Mat(2 * projected_points.size(), 1, CV_32F, projected_points.data());
    double total_error = cv::norm(x - y, cv::NORM_L2SQR);
    return total_error;
  };

  for (std::size_t i = 0; i < calibration_object_points.size(); i++) {
    std::vector<cv::Point2f> mini_opencv_projected_points;
    std::vector<cv::Point2f> opencv_projected_points;
    std::vector<cv::Point2f> ceres_projected_points;

    cv::projectPoints(
      calibration_object_points[i], mini_opencv_calibration_rvecs[i],
      mini_opencv_calibration_tvecs[i], mini_opencv_camera_matrix, mini_opencv_dist_coeffs,
      mini_opencv_projected_points);

    cv::projectPoints(
      calibration_object_points[i], opencv_calibration_rvecs[i], opencv_calibration_tvecs[i],
      opencv_camera_matrix, opencv_dist_coeffs, opencv_projected_points);

    cv::projectPoints(
      calibration_object_points[i], ceres_calibration_rvecs[i], ceres_calibration_tvecs[i],
      ceres_camera_matrix, ceres_dist_coeffs, ceres_projected_points);

    double mini_opencv_error =
      get_reprojection_error(mini_opencv_projected_points, calibration_image_points[i]);
    double opencv_error =
      get_reprojection_error(opencv_projected_points, calibration_image_points[i]);
    double ceres_error =
      get_reprojection_error(ceres_projected_points, calibration_image_points[i]);

    total_mini_opencv_calibration_error += mini_opencv_error;
    total_opencv_calibration_error += opencv_error;
    total_ceres_calibration_error += ceres_error;

    printf(
      "id=%lu | mini_opencv_error=%.4f | opencv_error=%.4f | ceres_error=%.4f\n", i,
      mini_opencv_error, opencv_error, ceres_error);
  }

  double rms_mini_opencv_calibration_error = std::sqrt(
    total_mini_opencv_calibration_error / (rows * cols * calibration_object_points.size()));
  double rms_opencv_calibration_error =
    std::sqrt(total_opencv_calibration_error / (rows * cols * calibration_object_points.size()));
  double rms_ceres_calibration_error =
    std::sqrt(total_ceres_calibration_error / (rows * cols * calibration_object_points.size()));

  printf(
    "summary | mini_opencv_error=%.4f | opencv_error=%.4f | ceres_error=%.4f\n",
    rms_mini_opencv_calibration_error, rms_opencv_calibration_error, rms_ceres_calibration_error);

  auto mini_opencv_duration =
    std::chrono::duration_cast<std::chrono::seconds>(mini_opencv_stop - mini_opencv_start);
  auto opencv_duration =
    std::chrono::duration_cast<std::chrono::seconds>(opencv_stop - opencv_start);
  auto ceres_duration = std::chrono::duration_cast<std::chrono::seconds>(ceres_stop - ceres_start);

  std::cout << "Mini opencv time: " << mini_opencv_duration.count() << " s" << std::endl;
  std::cout << "Opencv time: " << opencv_duration.count() << " s" << std::endl;
  std::cout << "Ceres time: " << ceres_duration.count() << " s" << std::endl;

  return 0;
}
