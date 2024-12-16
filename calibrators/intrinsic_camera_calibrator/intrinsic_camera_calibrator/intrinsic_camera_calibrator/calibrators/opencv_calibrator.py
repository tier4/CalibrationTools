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
from intrinsic_camera_calibrator.camera_models.opencv_camera_model import OpenCVCameraModel
from intrinsic_camera_calibrator.parameter import Parameter


class OpenCVCalibrator(Calibrator):
    """Wrapper of the opencv's camera calibration routine."""

    def __init__(self, lock: threading.RLock, cfg: Dict = {}):
        super().__init__(lock, cfg)
        self.radial_distortion_coefficients = Parameter(int, value=2, min_value=0, max_value=3)
        self.rational_distortion_coefficients = Parameter(int, value=0, min_value=0, max_value=3)
        self.use_tangential_distortion = Parameter(
            bool, value=True, min_value=False, max_value=True
        )
        self.enable_prism_model = Parameter(bool, value=False, min_value=False, max_value=True)
        self.fix_principal_point = Parameter(bool, value=False, min_value=False, max_value=True)
        self.fix_aspect_ratio = Parameter(bool, value=False, min_value=False, max_value=True)
        self.use_lu_decomposition = Parameter(bool, value=False, min_value=False, max_value=True)
        self.use_qr_decomposition = Parameter(bool, value=False, min_value=False, max_value=True)

        self.set_parameters(**cfg)

    def get_model_info(self) -> Tuple[Dict, CameraModelEnum]:
        with self.lock:
            return self.get_parameters_values(), CameraModelEnum.OPENCV

    def _calibration_impl(self, detections: List[BoardDetection]) -> OpenCVCameraModel:
        """Implement the calibrator interface."""
        height = detections[0].get_image_height()
        width = detections[0].get_image_width()

        camera_model = make_camera_model(camera_model_type=CameraModelEnum.OPENCV)
        with self.lock:
            camera_model.update_config(
                radial_distortion_coefficients=self.radial_distortion_coefficients.value,
                rational_distortion_coefficients=self.rational_distortion_coefficients.value,
                use_tangential_distortion=self.use_tangential_distortion.value,
                enable_prism_model=self.enable_prism_model.value,
                fix_principal_point=self.fix_principal_point.value,
                fix_aspect_ratio=self.fix_aspect_ratio.value,
                use_lu_decomposition=self.use_lu_decomposition.value,
                use_qr_decomposition=self.use_qr_decomposition.value,
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
