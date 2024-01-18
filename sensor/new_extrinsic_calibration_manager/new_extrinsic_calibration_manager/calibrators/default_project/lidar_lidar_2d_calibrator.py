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

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="default_project", calibrator_name="lidar_lidar_2d_calibrator"
)
class LidarLidar2DCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame: str = kwargs["base_frame"]
        self.source_frame: str = kwargs["source_frame"]
        self.target_frame: str = kwargs["target_frame"]

        self.required_frames.extend([self.base_frame, self.source_frame, self.target_frame])

        self.add_calibrator(
            service_name="calibrate_lidar_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.target_frame, child=self.source_frame),
            ],
        )
