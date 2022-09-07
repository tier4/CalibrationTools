#!/usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import cv2
from extrinsic_interactive_calibrator.utils import cv_to_transformation_matrix
from extrinsic_interactive_calibrator.utils import tf_message_to_transform_matrix
from extrinsic_interactive_calibrator.utils import transform_matrix_to_cv
import numpy as np


class Calibrator:
    def __init__(self):

        # Calibration parameters
        self.min_points = None
        self.inlier_error = None
        self.flags = None
        self.use_ransac = None
        self.ransac_iters = 200

        # Camera parameters
        self.k = None
        self.d = None
        pass

    def set_min_points(self, min_points):
        self.min_points = min_points

    def set_inlier_error(self, inlier_error):
        self.inlier_error = inlier_error

    def set_camera_info(self, k, d):
        self.k = np.array(k).reshape(3, 3)
        self.d = np.array(d).reshape(
            -1,
        )

    def set_method(self, method):

        if method == "sqpnp":
            self.flags = cv2.SOLVEPNP_SQPNP
        else:
            self.flags = None

    def set_ransac(self, use_ransac):
        self.use_ransac = use_ransac

    def calibrate(self, object_points, image_points):

        if len(object_points) == 0 or len(image_points) == 0:
            return None

        object_points = np.array(object_points, dtype=np.float64)
        image_points = np.array(image_points, dtype=np.float64)

        num_points, dim = object_points.shape
        assert dim == 3
        assert num_points == image_points.shape[0]

        if num_points < self.min_points:
            return None

        if self.use_ransac:
            return self.calibrate_ransac(object_points, image_points)

        tvec = np.zeros((3,))
        rvec = np.zeros((3, 3))

        try:
            retval, rvec, tvec = cv2.solvePnP(
                object_points, image_points, self.k, self.d, flags=self.flags
            )
        except Exception as e:
            pass

        camera_to_lidar_transform = cv_to_transformation_matrix(tvec, rvec)

        return camera_to_lidar_transform

    def calibrate_ransac(self, object_points, image_points):

        num_points, dim = object_points.shape

        best_tvec = np.zeros((3,))
        best_rvec = np.zeros((3, 3))
        best_inliers = -1
        best_error = np.inf

        for iter in range(self.ransac_iters):

            indexes = np.random.choice(num_points, min(num_points, self.min_points))
            object_points_iter = object_points[indexes, :]
            image_points_iter = image_points[indexes, :]

            try:
                retval, iter_rvec, iter_tvec = cv2.solvePnP(
                    object_points_iter, image_points_iter, self.k, self.d, flags=self.flags
                )
            except Exception as e:
                continue

            reproj_error_iter, inliers = self.calculate_reproj_error(
                object_points, image_points, tvec=iter_tvec, rvec=iter_rvec
            )

            if (
                inliers.sum() == best_inliers and reproj_error_iter < best_error
            ) or inliers.sum() > best_inliers:
                best_error = reproj_error_iter
                best_inliers = inliers.sum()
                best_tvec = iter_tvec
                best_rvec = iter_rvec

        camera_to_lidar_transform = cv_to_transformation_matrix(best_tvec, best_rvec)

        return camera_to_lidar_transform

    def calculate_reproj_error(
        self, object_points, image_points, tvec=None, rvec=None, tf_msg=None, transform_matrix=None
    ):

        if isinstance(object_points, list) and isinstance(image_points, list):
            if len(object_points) == 0:
                return 0.0, 0

            object_points = np.array(object_points, dtype=np.float64)
            image_points = np.array(image_points, dtype=np.float64)

        if tf_msg is not None:
            transform_matrix = tf_message_to_transform_matrix(tf_msg)

        if transform_matrix is not None:
            tvec, rvec = transform_matrix_to_cv(transform_matrix)

        assert tvec is not None
        assert rvec is not None
        num_points, dim = object_points.shape
        projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, self.k, self.d)
        projected_points = projected_points.reshape((num_points, 2))
        reproj_error = np.linalg.norm(projected_points - image_points, axis=1)

        if self.use_ransac:
            inliers = reproj_error <= self.inlier_error
        else:
            inliers = np.ones_like(reproj_error)

        return reproj_error.mean(), inliers
