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

from collections import defaultdict
from typing import Dict

import numpy as np

from sensor_calibration_manager.calibrator_base import CalibratorBase
from sensor_calibration_manager.calibrator_registry import CalibratorRegistry
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="x2", calibrator_name="mapping_based_lidar_lidar_calibrator"
)
class MappingBasedLidarLidarCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame = "base_link"
        self.top_sensor_kit_frame = "top_unit_base_link"
        self.front_sensor_kit_frame = "front_unit_base_link"
        self.rear_sensor_kit_frame = "rear_unit_base_link"

        self.mapping_lidar_frame = "pandar_40p_left"
        self.calibration_lidar_frames = [
            "pandar_qt_left",
            "pandar_40p_right",
            "pandar_qt_right",
            "pandar_40p_front",
            "pandar_qt_front",
            "pandar_40p_rear",
            "pandar_qt_rear",
        ]

        self.calibration_base_lidar_frames = [
            "pandar_qt_left",
            "pandar_40p_right",
            "pandar_qt_right",
            "pandar_40p_front",
            "pandar_qt_front",
            "pandar_40p_rear",
            "pandar_qt_rear",
        ]

        self.required_frames.extend(
            [
                self.base_frame,
                self.top_sensor_kit_frame,
                self.front_sensor_kit_frame,
                self.rear_sensor_kit_frame,
                self.mapping_lidar_frame,
                *self.calibration_lidar_frames,
                *self.calibration_base_lidar_frames,
            ]
        )

        self.add_calibrator(
            service_name="calibrate_lidar_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.mapping_lidar_frame, child=calibration_lidar_frame)
                for calibration_lidar_frame in self.calibration_lidar_frames
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        top_sensor_kit_to_main_lidar_transform = self.get_transform_matrix(
            self.top_sensor_kit_frame, self.mapping_lidar_frame
        )

        front_sensor_kit_to_main_lidar_transform = self.get_transform_matrix(
            self.front_sensor_kit_frame, "pandar_40p_front"
        )

        rear_sensor_kit_to_main_lidar_transform = self.get_transform_matrix(
            self.rear_sensor_kit_frame, "pandar_40p_rear"
        )

        base_to_mapping_lidar_transform = self.get_transform_matrix(
            self.base_frame, self.mapping_lidar_frame
        )

        calibration_lidar_to_base_lidar_transforms = {
            calibration_lidar_frame: self.get_transform_matrix(
                calibration_lidar_frame, calibration_base_lidar_frame
            )
            for calibration_lidar_frame, calibration_base_lidar_frame in zip(
                self.calibration_lidar_frames, self.calibration_base_lidar_frames
            )
        }

        results = defaultdict(lambda: defaultdict(np.array))

        # Top unit lidars
        results[self.top_sensor_kit_frame]["pandar_qt_left_base_link"] = (
            top_sensor_kit_to_main_lidar_transform
            @ calibration_transforms[self.mapping_lidar_frame]["pandar_qt_left"]
            @ calibration_lidar_to_base_lidar_transforms["pandar_qt_left"]
        )
        results[self.top_sensor_kit_frame]["pandar_40p_right_base_link"] = (
            top_sensor_kit_to_main_lidar_transform
            @ calibration_transforms[self.mapping_lidar_frame]["pandar_40p_right"]
            @ calibration_lidar_to_base_lidar_transforms["pandar_40p_right"]
        )
        results[self.top_sensor_kit_frame]["pandar_qt_right_base_link"] = (
            top_sensor_kit_to_main_lidar_transform
            @ calibration_transforms[self.mapping_lidar_frame]["pandar_qt_right"]
            @ calibration_lidar_to_base_lidar_transforms["pandar_qt_right"]
        )

        # Front unit lidars
        results[self.base_frame][self.front_sensor_kit_frame] = (
            base_to_mapping_lidar_transform
            @ calibration_transforms[self.mapping_lidar_frame]["pandar_40p_front"]
            @ np.linalg.inv(front_sensor_kit_to_main_lidar_transform)
        )
        results[self.front_sensor_kit_frame]["pandar_qt_front_base_link"] = (
            np.linalg.inv(results[self.base_frame][self.front_sensor_kit_frame])
            @ base_to_mapping_lidar_transform
            @ calibration_transforms[self.mapping_lidar_frame]["pandar_qt_front"]
            @ calibration_lidar_to_base_lidar_transforms["pandar_qt_front"]
        )

        # Rear unit lidars
        results[self.base_frame][self.rear_sensor_kit_frame] = (
            base_to_mapping_lidar_transform
            @ calibration_transforms[self.mapping_lidar_frame]["pandar_40p_rear"]
            @ np.linalg.inv(rear_sensor_kit_to_main_lidar_transform)
        )
        results[self.rear_sensor_kit_frame]["pandar_qt_rear_base_link"] = (
            np.linalg.inv(results[self.base_frame][self.rear_sensor_kit_frame])
            @ base_to_mapping_lidar_transform
            @ calibration_transforms[self.mapping_lidar_frame]["pandar_qt_rear"]
            @ calibration_lidar_to_base_lidar_transforms["pandar_qt_rear"]
        )

        return results
