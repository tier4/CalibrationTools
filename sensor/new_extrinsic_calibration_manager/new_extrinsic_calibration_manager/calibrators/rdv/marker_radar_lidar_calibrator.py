from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="rdv", calibrator_name="marker_radar_lidar_calibrator"
)
class MarkerRadarLidarCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.radar_parallel_frame = kwargs["radar_parallel_frame"]
        self.radar_frame = kwargs["radar_frame"]
        self.lidar_frame = kwargs["lidar_frame"]

        self.required_frames.extend([self.radar_parallel_frame, self.radar_frame, self.lidar_frame])

        print("RDV_MarkerRadarLidarCalibrator")

        self.add_calibrator(
            service_name="calibrate_radar_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.radar_frame, child=self.lidar_frame)
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        lidar_to_radar_parallel_transform = self.get_transform_matrix(
            self.lidar_frame, self.radar_parallel_frame
        )

        radar_parallel_to_radar_transform = np.linalg.inv(
            calibration_transforms[self.radar_frame][self.lidar_frame]
            @ lidar_to_radar_parallel_transform
        )

        results = {self.radar_parallel_frame: {self.radar_frame: radar_parallel_to_radar_transform}}

        return results
