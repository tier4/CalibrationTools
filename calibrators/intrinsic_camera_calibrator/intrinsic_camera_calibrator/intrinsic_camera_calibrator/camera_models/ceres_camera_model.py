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

import os
from typing import List
from typing import Optional

from ceres_intrinsic_camera_calibrator.ceres_intrinsic_camera_calibrator_py import calibrate
from intrinsic_camera_calibrator.camera_models.camera_model import CameraModel
from intrinsic_camera_calibrator.camera_models.opencv_camera_model import OpenCVCameraModel
import numpy as np


class CeresCameraModel(CameraModel):
    """Basic Ceres camera model class."""

    def __init__(
        self,
        k: Optional[np.array] = None,
        d: Optional[np.array] = None,
        height: Optional[int] = None,
        width: Optional[int] = None,
    ):
        super().__init__(k, d, height, width)
        self.radial_distortion_coefficients: Optional[int] = None
        self.rational_distortion_coefficients: Optional[int] = None
        self.use_tangential_distortion: Optional[bool] = None
        self.pre_calibration_num_samples: Optional[int] = None
        self.regularization_weight: Optional[float] = None
        self.verbose = (
            True if os.getenv("GLOG_minloglevel") == "0" else False
        )  # cSpell:ignore minloglevel

    def init_calibrate(
        self, object_points_list: List[np.array], image_points_list: List[np.array]
    ) -> OpenCVCameraModel:
        """Init calibrate of Ceres camera model."""
        camera_model = OpenCVCameraModel()
        camera_model.update_config(
            radial_distortion_coefficients=self.radial_distortion_coefficients,
            rational_distortion_coefficients=self.rational_distortion_coefficients,
            use_tangential_distortion=self.use_tangential_distortion,
        )

        # Consider only part of data (distributed uniformly)
        indices = np.round(
            np.linspace(
                0,
                len(object_points_list) - 1,
                np.min([self.pre_calibration_num_samples, len(object_points_list)]),
            )
        ).astype(int)
        partial_object_points_list = [object_points_list[i] for i in indices]
        partial_image_points_list = [image_points_list[i] for i in indices]

        camera_model.calibrate(
            height=self.height,
            width=self.width,
            object_points_list=partial_object_points_list,
            image_points_list=partial_image_points_list,
        )

        num_coeffs = 5 if self.rational_distortion_coefficients == 0 else 8
        if camera_model.d.size > num_coeffs:
            camera_model.d = camera_model.d.reshape(-1, camera_model.d.size)[:, :num_coeffs]

        return camera_model

    def _calibrate_impl(
        self, object_points_list: List[np.array], image_points_list: List[np.array]
    ):
        """Calibrate Ceres camera model."""
        camera_model = self.init_calibrate(object_points_list, image_points_list)

        _, camera_matrix, dist_coeffs, _, _ = calibrate(
            object_points_list=object_points_list,
            image_points_list=image_points_list,
            initial_camera_matrix=camera_model.k,
            initial_dist_coeffs=camera_model.d,
            num_radial_coeffs=self.radial_distortion_coefficients,
            num_rational_coeffs=self.rational_distortion_coefficients,
            use_tangential_distortion=self.use_tangential_distortion,
            regularization_weight=self.regularization_weight,
            verbose=self.verbose,
        )

        self.k = camera_matrix
        self.d = dist_coeffs

    def _update_config_impl(
        self,
        radial_distortion_coefficients: int,
        rational_distortion_coefficients: int,
        use_tangential_distortion: bool,
        pre_calibration_num_samples: int,
        regularization_weight: float,
        **kwargs
    ):
        """Update parameters."""
        self.radial_distortion_coefficients = radial_distortion_coefficients
        self.rational_distortion_coefficients = rational_distortion_coefficients
        self.use_tangential_distortion = use_tangential_distortion
        self.pre_calibration_num_samples = pre_calibration_num_samples
        self.regularization_weight = regularization_weight
