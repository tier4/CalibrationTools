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


from intrinsic_camera_calibrator.calibrators.calibrator import Calibrator
from intrinsic_camera_calibrator.calibrators.calibrator import CalibratorEnum
from intrinsic_camera_calibrator.calibrators.ceres_calibrator import CeresCalibrator
from intrinsic_camera_calibrator.calibrators.opencv_calibrator import OpenCVCalibrator


def make_calibrator(calibrator_type: CalibratorEnum, **kwargs) -> Calibrator:
    """Create a calibrator using a factory design pattern."""
    classes_dic = {
        CalibratorEnum.OPENCV: OpenCVCalibrator,
        CalibratorEnum.CERES: CeresCalibrator,
    }
    return classes_dic[calibrator_type](**kwargs)
