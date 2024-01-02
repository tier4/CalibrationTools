# from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="default_project", calibrator_name="ground_plane_calibrator"
)
class GroundPlaneCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame: str = kwargs["base_frame"]
        self.lidar_frame: str = kwargs["lidar_frame"]

        self.required_frames.extend([self.base_frame, self.lidar_frame])

        print("DefaultProject_GroundPlane2DCalibrator")

        self.add_calibrator(
            service_name="calibrate_base_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.base_frame, child=self.lidar_frame),
            ],
        )
