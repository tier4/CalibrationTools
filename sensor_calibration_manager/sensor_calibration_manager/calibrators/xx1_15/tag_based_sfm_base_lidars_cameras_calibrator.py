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


import logging
from typing import Dict
from typing import List
from typing import Optional

import numpy as np

from sensor_calibration_manager.calibrator_base import CalibratorBase
from sensor_calibration_manager.calibrator_registry import CalibratorRegistry
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="xx1_15", calibrator_name="tag_based_sfm_base_lidars_cameras_calibrator"
)
class TagBasedSfmBaseLidarsCamerasCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame = kwargs["base_frame"]
        self.sensor_kit_frame = "sensor_kit_base_link"

        self.main_sensor_frame = kwargs["main_calibration_sensor_frame"]

        self.calibration_camera_optical_link_frames: List[str] = [
            kwargs["calibration_camera_0_frame"],
            kwargs["calibration_camera_1_frame"],
            kwargs["calibration_camera_2_frame"],
            kwargs["calibration_camera_3_frame"],
            kwargs["calibration_camera_4_frame"],
            kwargs["calibration_camera_5_frame"],
            kwargs["calibration_camera_6_frame"],
        ]
        self.calibration_camera_link_frames = [
            camera_frame.replace("camera_optical_link", "camera_link")
            for camera_frame in self.calibration_camera_optical_link_frames
        ]

        self.required_frames.extend(
            [
                self.base_frame,
                self.sensor_kit_frame,
                self.main_sensor_frame,
                *self.calibration_camera_optical_link_frames,
                *self.calibration_camera_link_frames,
            ]
        )

        self.add_calibrator(
            service_name="calibrate_base_lidars_cameras",
            expected_calibration_frames=[
                FramePair(parent=self.main_sensor_frame, child=self.base_frame),
                *[
                    FramePair(parent=self.main_sensor_frame, child=calibration_frame)
                    for calibration_frame in self.calibration_camera_optical_link_frames
                ],
            ],
        )

        self.cached_constant_transforms = False
        self.cached_sensor_kit_to_main_sensor_transform: Optional[np.array] = None
        self.cached_optical_link_to_camera_link_transforms: Optional[List[np.array]] = None

    def on_check_tf_timer(self):
        super().on_check_tf_timer()

        if self.tfs_ready and not self.cached_constant_transforms:
            self.cache_constant_transforms()

    def cache_constant_transforms(self):
        """Cache the constant tfs needed by `post_process` before calibrating.

        `post_process` runs after the calibration has finished, at which point the
        calibrator node is already broadcasting the optimized sensor poses for
        visualization purposes. Since tf2 only allows a single parent per frame, those
        broadcasts re-parent the calibration frames, and any query whose path traverses
        one of them returns a value containing the inverse of the optimized poses,
        which cancels out the calibration results during `post_process`. To avoid this,
        the constant tfs are cached here, as soon as they become available and before
        any calibration result can be broadcast.
        """
        self.cached_sensor_kit_to_main_sensor_transform = self.get_transform_matrix(
            self.sensor_kit_frame, self.main_sensor_frame
        )
        self.cached_optical_link_to_camera_link_transforms = [
            self.get_transform_matrix(camera_optical_link_frame, camera_link_frame)
            for camera_optical_link_frame, camera_link_frame in zip(
                self.calibration_camera_optical_link_frames, self.calibration_camera_link_frames
            )
        ]
        self.cached_constant_transforms = True
        logging.info("Cached the constant tfs used by post_process")

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        if not self.cached_constant_transforms:
            logging.warning("The constant tfs were not cached. Falling back to a live query")
            self.cache_constant_transforms()

        sensor_kit_to_mapping_lidar_transform = self.cached_sensor_kit_to_main_sensor_transform
        optical_link_to_camera_link_transforms = self.cached_optical_link_to_camera_link_transforms

        base_to_top_sensor_kit_transform = np.linalg.inv(
            sensor_kit_to_mapping_lidar_transform
            @ calibration_transforms[self.main_sensor_frame][self.base_frame]
        )
        results = {self.base_frame: {self.sensor_kit_frame: base_to_top_sensor_kit_transform}}
        results[self.sensor_kit_frame] = {}

        for camera_frame, optical_link_to_camera_link_transform in zip(
            self.calibration_camera_optical_link_frames, optical_link_to_camera_link_transforms
        ):
            results[self.sensor_kit_frame][
                camera_frame.replace("camera_optical_link", "camera_link")
            ] = (
                sensor_kit_to_mapping_lidar_transform
                @ calibration_transforms[self.main_sensor_frame][camera_frame]
                @ optical_link_to_camera_link_transform
            )

        return results
