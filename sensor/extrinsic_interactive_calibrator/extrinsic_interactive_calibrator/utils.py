#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc.
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

import array
from copy import deepcopy

import cv2
import numpy as np
from sensor_msgs.msg import CameraInfo


def get_calibration_flags(
    fix_principal_point=False, fix_aspect_ratio=False, zero_tangent_dist=True, num_ks=2
):
    calib_flags = 0

    if fix_principal_point:
        calib_flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    if fix_aspect_ratio:
        calib_flags |= cv2.CALIB_FIX_ASPECT_RATIO
    if zero_tangent_dist:
        calib_flags |= cv2.CALIB_ZERO_TANGENT_DIST
    if num_ks > 3:
        calib_flags |= cv2.CALIB_RATIONAL_MODEL
    if num_ks < 6:
        calib_flags |= cv2.CALIB_FIX_K6
    if num_ks < 5:
        calib_flags |= cv2.CALIB_FIX_K5
    if num_ks < 4:
        calib_flags |= cv2.CALIB_FIX_K4
    if num_ks < 3:
        calib_flags |= cv2.CALIB_FIX_K3
    if num_ks < 2:
        calib_flags |= cv2.CALIB_FIX_K2
    if num_ks < 1:
        calib_flags |= cv2.CALIB_FIX_K1
    calib_flags |= cv2.CALIB_USE_INTRINSIC_GUESS

    return calib_flags


def camera_lidar_calibrate_intrinsics(
    object_points: np.array, image_points: np.array, initial_camera_info: CameraInfo
):
    object_points = object_points.astype(np.float32)
    image_points = image_points.astype(np.float32)

    num_object_points, object_dim = object_points.shape
    num_image_points, image_dim = image_points.shape

    assert num_object_points == num_image_points
    assert object_dim == 3
    assert image_dim == 2

    initial_k = np.array(initial_camera_info.k).reshape(3, 3)
    initial_d = np.array(initial_camera_info.d).flatten()

    calib_flags = get_calibration_flags()

    _, new_k, new_d, _, _ = cv2.calibrateCamera(
        [object_points.reshape(-1, 3)],
        [image_points.reshape(-1, 1, 2)],
        (initial_camera_info.width, initial_camera_info.height),
        cameraMatrix=initial_k,
        distCoeffs=initial_d,
        flags=calib_flags,
    )

    optimized_camera_info = deepcopy(initial_camera_info)
    optimized_camera_info.k = new_k.reshape(-1)
    optimized_camera_info.d = array.array("d", new_d)

    ncm, _ = cv2.getOptimalNewCameraMatrix(
        np.array(optimized_camera_info.k).reshape(3, 3),
        np.array(optimized_camera_info.d).reshape(-1),
        (optimized_camera_info.width, optimized_camera_info.height),
        0.0,
    )

    p = np.zeros((3, 4), dtype=np.float64)

    for j in range(3):
        for i in range(3):
            p[j, i] = ncm[j, i]

    optimized_camera_info.p = p.reshape(-1)
    return optimized_camera_info
