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

from sensor_calibration_manager.calibrator_base import CalibratorBase
from sensor_calibration_manager.calibrator_registry import CalibratorRegistry
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="default_project", calibrator_name="tag_based_sfm_base_lidars_cameras_calibrator"
)
class TagBasedSfmBaseLidarsCamerasCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame = kwargs["base_frame"]

        self.main_sensor_frame = kwargs["main_calibration_sensor_frame"]
        self.calibration_lidar_frames = [
            kwargs["calibration_lidar_1_frame"],
            kwargs["calibration_lidar_2_frame"],
            kwargs["calibration_lidar_3_frame"],
        ]

        self.calibration_camera_optical_link_frames = [
            kwargs["calibration_camera_0_frame"],
            kwargs["calibration_camera_1_frame"],
            kwargs["calibration_camera_2_frame"],
            kwargs["calibration_camera_3_frame"],
            kwargs["calibration_camera_4_frame"],
            kwargs["calibration_camera_5_frame"],
            kwargs["calibration_camera_6_frame"],
        ]

        self.required_frames.extend(
            [
                self.base_frame,
                self.main_sensor_frame,
                *self.calibration_lidar_frames,
                *self.calibration_camera_optical_link_frames,
            ]
        )

        self.add_calibrator(
            service_name="calibrate_base_lidars_cameras",
            expected_calibration_frames=[
                FramePair(parent=self.main_sensor_frame, child=self.base_frame),
                *[
                    FramePair(parent=self.main_sensor_frame, child=calibration_frame)
                    for calibration_frame in self.calibration_lidar_frames
                ],
                *[
                    FramePair(parent=self.main_sensor_frame, child=calibration_frame)
                    for calibration_frame in self.calibration_camera_optical_link_frames
                ],
            ],
        )
