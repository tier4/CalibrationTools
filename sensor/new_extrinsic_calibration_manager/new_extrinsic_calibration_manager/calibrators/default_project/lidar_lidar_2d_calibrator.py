# from typing import Dict

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

        print("DefaultProject_LidarLidar2DCalibrator")

        self.add_calibrator(
            service_name="calibrate_lidar_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.target_frame, child=self.source_frame),
            ],
        )
