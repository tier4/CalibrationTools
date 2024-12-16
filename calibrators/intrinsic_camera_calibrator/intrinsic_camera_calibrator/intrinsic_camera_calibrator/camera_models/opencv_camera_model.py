#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

from typing import List
from typing import Optional

import cv2
from intrinsic_camera_calibrator.camera_models.camera_model import CameraModel
from intrinsic_camera_calibrator.utils import toggle_flag
import numpy as np


class OpenCVCameraModel(CameraModel):
    """Basic opencv's camera model class."""

    def __init__(
        self,
        k: Optional[np.array] = None,
        d: Optional[np.array] = None,
        height: Optional[int] = None,
        width: Optional[int] = None,
    ):
        super().__init__(k, d, height, width)
        self.flags = 0

    def _calibrate_impl(
        self, object_points_list: List[np.array], image_points_list: List[np.array]
    ):
        """Calibrate OpenCV camera model."""
        object_points_list = [
            object_points.astype(np.float32).reshape(-1, 3) for object_points in object_points_list
        ]
        image_points_list = [
            image_points.astype(np.float32).reshape(-1, 1, 2) for image_points in image_points_list
        ]

        _, self.k, self.d, rvecs, tvecs = cv2.calibrateCamera(
            object_points_list,
            image_points_list,
            (self.width, self.height),
            cameraMatrix=None,
            distCoeffs=None,
            flags=self.flags,
        )
        pass

    def _update_config_impl(
        self,
        radial_distortion_coefficients: int,
        rational_distortion_coefficients: int,
        use_tangential_distortion: bool,
        enable_prism_model: bool = False,
        fix_principal_point: bool = False,
        fix_aspect_ratio: bool = False,
        use_lu_decomposition: bool = False,
        use_qr_decomposition: bool = False,
        **kwargs
    ):
        """Update parameters."""
        for idx, k in enumerate([cv2.CALIB_FIX_K1, cv2.CALIB_FIX_K2, cv2.CALIB_FIX_K3]):
            self.flags = toggle_flag(self.flags, k, not (idx < radial_distortion_coefficients))

        for idx, k in enumerate([cv2.CALIB_FIX_K4, cv2.CALIB_FIX_K5, cv2.CALIB_FIX_K6]):
            self.flags = toggle_flag(self.flags, k, not (idx < rational_distortion_coefficients))
        self.flags = toggle_flag(
            self.flags, cv2.CALIB_RATIONAL_MODEL, rational_distortion_coefficients > 0
        )

        self.flags = toggle_flag(self.flags, cv2.CALIB_THIN_PRISM_MODEL, enable_prism_model)
        self.flags = toggle_flag(self.flags, cv2.CALIB_FIX_PRINCIPAL_POINT, fix_principal_point)
        self.flags = toggle_flag(self.flags, cv2.CALIB_FIX_ASPECT_RATIO, fix_aspect_ratio)
        self.flags = toggle_flag(self.flags, cv2.CALIB_USE_LU, use_lu_decomposition)
        self.flags = toggle_flag(self.flags, cv2.CALIB_USE_QR, use_qr_decomposition)
        self.flags = toggle_flag(
            self.flags, cv2.CALIB_ZERO_TANGENT_DIST, not use_tangential_distortion
        )
