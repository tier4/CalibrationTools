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

from typing import Dict

import numpy as np

from sensor_calibration_manager.calibrator_base import CalibratorBase
from sensor_calibration_manager.calibrator_registry import CalibratorRegistry
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="x1", calibrator_name="ground_plane_calibrator"
)
class GroundPlaneCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame = "base_link"
        self.sensor_kit_frame = "sensor_kit_base_link"
        self.lidar_frame = "velodyne_top"

        self.required_frames.extend([self.base_frame, self.sensor_kit_frame, self.lidar_frame])

        self.add_calibrator(
            service_name="calibrate_base_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.base_frame, child=self.lidar_frame),
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        base_to_lidar_transform = calibration_transforms[self.base_frame][self.lidar_frame]

        sensor_kit_to_lidar_transform = self.get_transform_matrix(
            self.sensor_kit_frame, self.lidar_frame
        )

        base_to_sensor_kit_transform = base_to_lidar_transform @ np.linalg.inv(
            sensor_kit_to_lidar_transform
        )

        result = {self.base_frame: {self.sensor_kit_frame: base_to_sensor_kit_transform}}

        return result
