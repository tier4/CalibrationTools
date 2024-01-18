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
    project_name="rdv", calibrator_name="tag_based_pnp_calibrator"
)
class TagBasedPNPCalibrator(CalibratorBase):
    required_frames = ["sensor_kit_base_link", "pandar_top_base_link", "pandar_top"]

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.camera_name = kwargs["camera_name"]
        self.required_frames.append(f"{self.camera_name}/camera_link")
        self.required_frames.append(f"{self.camera_name}/camera_optical_link")

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent=f"{self.camera_name}/camera_optical_link", child="pandar_top"),
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        camera_to_lidar_transform = calibration_transforms[
            f"{self.camera_name}/camera_optical_link"
        ]["pandar_top"]

        sensor_kit_to_lidar_transform = self.get_transform_matrix(
            "sensor_kit_base_link", "pandar_top"
        )

        camera_to_optical_link_transform = self.get_transform_matrix(
            f"{self.camera_name}/camera_link", f"{self.camera_name}/camera_optical_link"
        )

        sensor_kit_camera_link_transform = np.linalg.inv(
            camera_to_optical_link_transform
            @ camera_to_lidar_transform
            @ np.linalg.inv(sensor_kit_to_lidar_transform)
        )

        result = {
            "sensor_kit_base_link": {
                f"{self.camera_name}/camera_link": sensor_kit_camera_link_transform
            }
        }
        return result
