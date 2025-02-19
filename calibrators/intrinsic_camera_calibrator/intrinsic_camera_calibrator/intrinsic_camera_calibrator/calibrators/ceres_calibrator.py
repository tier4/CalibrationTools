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


import threading
from typing import Dict
from typing import List
from typing import Tuple

from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.calibrators.calibrator import Calibrator
from intrinsic_camera_calibrator.camera_models.camera_model import CameraModelEnum
from intrinsic_camera_calibrator.camera_models.camera_model_factory import make_camera_model
from intrinsic_camera_calibrator.camera_models.ceres_camera_model import CeresCameraModel
from intrinsic_camera_calibrator.parameter import Parameter


class CeresCalibrator(Calibrator):
    def __init__(self, lock: threading.RLock, cfg: Dict = {}):
        super().__init__(lock, cfg)
        self.radial_distortion_coefficients = Parameter(int, value=2, min_value=0, max_value=3)
        self.rational_distortion_coefficients = Parameter(int, value=0, min_value=0, max_value=3)
        self.use_tangential_distortion = Parameter(
            bool, value=True, min_value=False, max_value=True
        )
        self.pre_calibration_num_samples = Parameter(int, value=40, min_value=1, max_value=100)
        self.coeffs_regularization_weight = Parameter(
            float, value=0.001, min_value=0.0, max_value=1.0
        )
        self.fov_regularization_weight = Parameter(float, value=0.0, min_value=0.0, max_value=1.0)

        self.set_parameters(**cfg)

    def get_model_info(self) -> Tuple[Dict, CameraModelEnum]:
        with self.lock:
            return self.get_parameters_values(), CameraModelEnum.CERES

    def _calibration_impl(self, detections: List[BoardDetection]) -> CeresCameraModel:
        """Implement the calibrator interface."""
        height = detections[0].get_image_height()
        width = detections[0].get_image_width()

        camera_model = make_camera_model(camera_model_type=CameraModelEnum.CERES)
        with self.lock:
            camera_model.update_config(
                radial_distortion_coefficients=self.radial_distortion_coefficients.value,
                rational_distortion_coefficients=self.rational_distortion_coefficients.value,
                use_tangential_distortion=self.use_tangential_distortion.value,
                pre_calibration_num_samples=self.pre_calibration_num_samples.value,
                coeffs_regularization_weight=self.coeffs_regularization_weight.value,
                fov_regularization_weight=self.fov_regularization_weight.value,
            )
        camera_model.calibrate(
            height=height,
            width=width,
            object_points_list=[
                detection.get_flattened_object_points() for detection in detections
            ],
            image_points_list=[detection.get_flattened_image_points() for detection in detections],
        )

        return camera_model
