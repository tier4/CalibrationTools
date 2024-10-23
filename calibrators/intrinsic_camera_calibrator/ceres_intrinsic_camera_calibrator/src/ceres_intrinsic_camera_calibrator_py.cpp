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

#include <Eigen/Core>
#include <Eigen/Dense>
#include <ceres_intrinsic_camera_calibrator/ceres_camera_intrinsics_optimizer.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <algorithm>
#include <iostream>
#include <tuple>
#include <vector>

#define STRINGIFY(x) #x
#define MACRO_STRINGIFY(x) STRINGIFY(x)

int add(int i, int j) { return i + j; }

std::tuple<double, double> test(const Eigen::MatrixXd & matrix)
{
  return std::make_tuple(matrix.determinant(), matrix.sum());
}

std::tuple<
  double, Eigen::MatrixXd, Eigen::MatrixXd, std::vector<Eigen::Vector3d>,
  std::vector<Eigen::Vector3d>>
calibrate(
  const std::vector<Eigen::MatrixXd> & object_points_eigen_list,
  const std::vector<Eigen::MatrixXd> & image_points_eigen_list,
  const Eigen::MatrixXd & initial_camera_matrix_eigen,
  const Eigen::MatrixXd & initial_dist_coeffs_eigen, int num_radial_coeffs, int num_rational_coeffs,
  bool use_tangential_distortion, bool verbose)
{
  if (
    initial_camera_matrix_eigen.cols() != 3 || initial_camera_matrix_eigen.rows() != 3 ||
    object_points_eigen_list.size() != image_points_eigen_list.size() || num_radial_coeffs < 0 ||
    num_radial_coeffs > 3 || num_rational_coeffs < 0 || num_rational_coeffs > 3 ||
    std::min<std::size_t>(initial_dist_coeffs_eigen.rows(), initial_dist_coeffs_eigen.cols()) > 1) {
    std::cout << "Invalid parameters" << std::endl;
    std::cout << "\t object_points_list.size(): " << object_points_eigen_list.size() << std::endl;
    std::cout << "\t image_points_list.size(): " << image_points_eigen_list.size() << std::endl;
    std::cout << "\t initial_camera_matrix:\n" << initial_camera_matrix_eigen << std::endl;
    std::cout << "\t initial_dist_coeffs:\n" << initial_dist_coeffs_eigen << std::endl;
    std::cout << "\t num_radial_coeffs: " << num_radial_coeffs << std::endl;
    std::cout << "\t num_rational_coeffs: " << num_rational_coeffs << std::endl;
    std::cout << "\t use_tangential_distortion: " << use_tangential_distortion << std::endl;
    return std::tuple<
      double, Eigen::MatrixXd, Eigen::MatrixXd, std::vector<Eigen::Vector3d>,
      std::vector<Eigen::Vector3d>>();
  }

  // Convert all the data to formats we are used to
  cv::Mat_<double> initial_camera_matrix_cv = cv::Mat_<double>::zeros(3, 3);
  cv::Mat_<double> initial_dist_coeffs_cv = cv::Mat_<double>::zeros(5, 1);
  cv::Mat_<double> camera_matrix_cv = cv::Mat_<double>::zeros(3, 3);
  cv::Mat_<double> dist_coeffs_cv = cv::Mat_<double>::zeros(5, 1);
  std::vector<cv::Mat> initial_rvecs_cv, rvecs_cv;
  std::vector<cv::Mat> initial_tvecs_cv, tvecs_cv;

  cv::eigen2cv(initial_camera_matrix_eigen, initial_camera_matrix_cv);
  cv::eigen2cv(initial_dist_coeffs_eigen, initial_dist_coeffs_cv);

  std::vector<std::vector<cv::Point3f>> object_points_list_cv;
  std::vector<std::vector<cv::Point2f>> image_points_list_cv;

  for (std::size_t view_id = 0; view_id < object_points_eigen_list.size(); view_id++) {
    std::vector<cv::Point3f> object_points;
    std::vector<cv::Point2f> image_points;

    const auto & input_object_points = object_points_eigen_list[view_id];
    const auto & input_image_points = image_points_eigen_list[view_id];

    object_points.reserve(input_object_points.rows());
    image_points.reserve(input_image_points.rows());

    for (int i = 0; i < input_image_points.rows(); i++) {
      object_points.emplace_back(
        input_object_points(i, 0), input_object_points(i, 1), input_object_points(i, 2));
      image_points.emplace_back(input_image_points(i, 0), input_image_points(i, 1));
    }

    object_points_list_cv.push_back(object_points);
    image_points_list_cv.push_back(image_points);
  }

  // Use PnP to get the initial board poses
  for (std::size_t view_id = 0; view_id < object_points_list_cv.size(); view_id++) {
    std::vector<cv::Point2f> calibration_projected_points;
    std::vector<cv::Point2f> pnp_projected_points;

    cv::Mat rvec, tvec;
    bool status = cv::solvePnP(
      object_points_list_cv[view_id], image_points_list_cv[view_id], initial_camera_matrix_cv,
      initial_dist_coeffs_cv, rvec, tvec);
    CV_UNUSED(status);
    initial_rvecs_cv.push_back(rvec);
    initial_tvecs_cv.push_back(tvec);
  }

  // Calibrate
  CeresCameraIntrinsicsOptimizer optimizer;
  optimizer.setRadialDistortionCoefficients(num_radial_coeffs);
  optimizer.setTangentialDistortion(use_tangential_distortion);
  optimizer.setRationalDistortionCoefficients(num_rational_coeffs);
  optimizer.setVerbose(verbose);
  optimizer.setData(
    initial_camera_matrix_cv, initial_dist_coeffs_cv, object_points_list_cv, image_points_list_cv,
    initial_rvecs_cv, initial_tvecs_cv);
  optimizer.dataToPlaceholders();
  optimizer.evaluate();
  optimizer.solve();
  optimizer.placeholdersToData();
  optimizer.evaluate();
  double rms_error = optimizer.getSolution(camera_matrix_cv, dist_coeffs_cv, rvecs_cv, tvecs_cv);

  // Extract the results
  Eigen::MatrixXd camera_matrix_eigen, dist_coeffs_eigen;
  std::vector<Eigen::Vector3d> rvecs_eigen, tvecs_eigen;

  cv::cv2eigen(camera_matrix_cv, camera_matrix_eigen);
  cv::cv2eigen(dist_coeffs_cv, dist_coeffs_eigen);

  rvecs_eigen.resize(rvecs_cv.size());
  tvecs_eigen.resize(tvecs_cv.size());

  for (std::size_t view_id = object_points_list_cv.size(); view_id < object_points_list_cv.size();
       view_id++) {
    cv::cv2eigen(rvecs_cv[view_id], rvecs_eigen[view_id]);
    cv::cv2eigen(tvecs_cv[view_id], tvecs_eigen[view_id]);
  }

  return std::make_tuple(
    rms_error, camera_matrix_eigen, dist_coeffs_eigen, rvecs_eigen, tvecs_eigen);
}

namespace py = pybind11;

PYBIND11_MODULE(ceres_intrinsic_camera_calibrator_py, m)
{
  // cSpell:ignore pbdoc,currentmodule,autosummary,toctree
  m.doc() = R"pbdoc(
        Ceres-based camera intrinsics calibrator module
        -----------------------

        .. currentmodule:: ceres_intrinsic_camera_calibrator_py

        .. autosummary::
           :toctree: _generate

           calibrate
    )pbdoc";

  m.def(
    "calibrate", calibrate,
    R"pbdoc(
        Calibrate camera intrinsics

        Args:
            object_points_list (List[np.array]): The object points from different views
            image_points_list (List[np.array]): The image points from different views
            initial_camera_matrix (np.array): The initial camera matrix
            initial_dist_coeffs (np.array): The initial distortion coefficients
            num_radial_coeffs (int): The number of radial distortion coefficients used during calibration
            num_rational_coeffs (int): The number of rational distortion coefficients used during calibration
            use_tangential_distortion (bool): Whether we should use tangential distortion during calibration
            verbose (bool): Whether we should print debug information

        Returns:
            The RMS reprojection error, the optimized camera intrinsics, and the board extrinsics
      )pbdoc",
    py::arg("object_points_list"), py::arg("image_points_list"), py::arg("initial_camera_matrix"),
    py::arg("initial_dist_coeffs"), py::arg("num_radial_coeffs"), py::arg("num_rational_coeffs"),
    py::arg("use_tangential_distortion"), py::arg("verbose") = false);

#ifdef VERSION_INFO
  m.attr("__version__") = MACRO_STRINGIFY(VERSION_INFO);
#else
  m.attr("__version__") = "dev";
#endif
}
