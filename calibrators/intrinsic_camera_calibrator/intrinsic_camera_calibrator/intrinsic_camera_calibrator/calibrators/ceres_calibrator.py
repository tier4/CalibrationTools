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

from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.calibrators.calibrator import Calibrator
from intrinsic_camera_calibrator.camera_model import CameraModel
from intrinsic_camera_calibrator.parameter import Parameter


class CeresCalibrator(Calibrator):
    def __init__(self, lock: threading.RLock, cfg: Dict = {}):
        super().__init__(lock, cfg)

        self.some_parameter_name = Parameter(int, value=2, min_value=0, max_value=6)
        self.set_parameters(**cfg)

    def _calibration_impl(self, detections: List[BoardDetection]) -> CameraModel:
        raise NotImplementedError
