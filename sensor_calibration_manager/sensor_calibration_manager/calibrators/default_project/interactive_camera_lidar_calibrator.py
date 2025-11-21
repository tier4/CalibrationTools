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


from typing import Dict

import numpy as np

from sensor_calibration_manager.calibrator_base import CalibratorBase
from sensor_calibration_manager.calibrator_registry import CalibratorRegistry
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="default_project", calibrator_name="interactive_camera_lidar_calibrator"
)
class InteractiveCameraLidarCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.camera_optical_link_frame = kwargs["camera_optical_link_frame"]
        self.lidar_frame = kwargs["lidar_frame"]

        self.required_frames.append(self.camera_optical_link_frame)
        self.required_frames.append(self.lidar_frame)

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.camera_optical_link_frame, child=self.lidar_frame),
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        optical_link_to_lidar_transform = calibration_transforms[self.camera_optical_link_frame][
            self.lidar_frame
        ]

        result = {
            self.lidar_frame: {
                self.camera_optical_link_frame: np.linalg.inv(optical_link_to_lidar_transform)
            }
        }
        return result
