from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="xx1", calibrator_name="ground_plane_calibrator"
)
class GroundPlaneCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame = "base_link"
        self.sensor_kit_frame = "sensor_kit_base_link"
        self.lidar_frame = "velodyne_top"

        self.required_frames.extend([self.base_frame, self.sensor_kit_frame, self.lidar_frame])

        print("XX1_GroundPlane2DCalibrator")

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
