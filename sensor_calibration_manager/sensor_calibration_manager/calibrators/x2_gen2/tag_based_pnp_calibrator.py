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
    project_name="x2_gen2", calibrator_name="tag_based_pnp_calibrator"
)
class TagBasedPNPCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.camera_name = kwargs["camera_name"]
        self.lidar_frame = kwargs["lidar_frame"]

        camera_name_to_sensor_kit_frame = {
            "camera0": "sensor_kit_base_link",
            "camera1": "sensor_kit_base_link",
            "camera2": "sensor_kit_base_link",
            "camera3": "sensor_kit_base_link",
            "camera4": "sensor_kit_base_link",
            "camera5": "sensor_kit_base_link",
            "camera6": "sensor_kit_base_link",
            "camera7": "sensor_kit_base_link",
        }

        camera_name_to_frame = {
            "camera0": "top_rear_center",
            "camera1": "left_center",
            "camera2": "right_center",
            "camera3": "top_front_right",
            "camera4": "top_front_center_right",
            "camera5": "top_front_left",
            "camera6": "left_front",
            "camera7": "right_front",
        }

        self.sensor_kit_frame = camera_name_to_sensor_kit_frame[self.camera_name]
        self.camera_frame = camera_name_to_frame[self.camera_name]

        self.required_frames.extend(
            [
                self.lidar_frame,
                self.sensor_kit_frame,
                f"{self.camera_frame}/camera_link",
                f"{self.camera_frame}/camera_optical_link",
            ]
        )

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent=f"{self.camera_frame}/camera_optical_link", child=self.lidar_frame),
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        optical_link_to_lidar_transform = calibration_transforms[
            f"{self.camera_frame}/camera_optical_link"
        ][self.lidar_frame]

        sensor_kit_to_lidar_transform = self.get_transform_matrix(
            self.sensor_kit_frame, self.lidar_frame
        )

        camera_to_optical_link_transform = self.get_transform_matrix(
            f"{self.camera_frame}/camera_link", f"{self.camera_frame}/camera_optical_link"
        )

        sensor_kit_camera_link_transform = np.linalg.inv(
            camera_to_optical_link_transform
            @ optical_link_to_lidar_transform
            @ np.linalg.inv(sensor_kit_to_lidar_transform)
        )

        result = {
            self.sensor_kit_frame: {
                f"{self.camera_frame}/camera_link": sensor_kit_camera_link_transform
            }
        }
        return result
