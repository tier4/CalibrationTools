from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="rdv", calibrator_name="tag_based_sfm_base_lidars_calibrator"
)
class TagBasedSfmBaseLidarsCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame = kwargs["base_frame"]
        self.sensor_kit_frame = "sensor_kit_base_link"

        self.main_sensor_frame = kwargs["main_calibration_sensor_frame"]
        self.calibration_lidar_frames = [
            kwargs["calibration_lidar_1_frame"],
            kwargs["calibration_lidar_2_frame"],
            kwargs["calibration_lidar_3_frame"],
        ]
        self.calibration_lidar_base_frames = [
            lidar_frame + "_base_link" for lidar_frame in self.calibration_lidar_frames
        ]

        self.required_frames.extend(
            [
                self.base_frame,
                self.sensor_kit_frame,
                self.main_sensor_frame,
                *self.calibration_lidar_frames,
                *self.calibration_lidar_base_frames,
            ]
        )

        print("RDV_tagBasedSfmBaseLidarsCalibrator")

        self.add_calibrator(
            service_name="calibrate_base_lidars",
            expected_calibration_frames=[
                FramePair(parent=self.main_sensor_frame, child=self.base_frame),
                *[
                    FramePair(parent=self.main_sensor_frame, child=calibration_frame)
                    for calibration_frame in self.calibration_lidar_frames
                ],
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        sensor_kit_to_mapping_lidar_transform = self.get_transform_matrix(
            self.sensor_kit_frame, self.main_sensor_frame
        )

        lidar_to_lidar_base_transforms = [
            self.get_transform_matrix(lidar_frame, lidar_base_frame)
            for lidar_frame, lidar_base_frame in zip(
                self.calibration_lidar_frames, self.calibration_lidar_base_frames
            )
        ]

        base_to_top_sensor_kit_transform = np.linalg.inv(
            sensor_kit_to_mapping_lidar_transform
            @ calibration_transforms[self.main_sensor_frame][self.base_frame]
        )
        results = {self.base_frame: {self.sensor_kit_frame: base_to_top_sensor_kit_transform}}
        results[self.sensor_kit_frame] = {}

        for lidar_frame, lidar_to_lidar_base_transform in zip(
            self.calibration_lidar_frames, lidar_to_lidar_base_transforms
        ):
            results[self.sensor_kit_frame][lidar_frame + "_base_link"] = (
                sensor_kit_to_mapping_lidar_transform
                @ calibration_transforms[self.main_sensor_frame][lidar_frame]
                @ lidar_to_lidar_base_transform
            )

        return results