# from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair

# import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="default_project", calibrator_name="tag_based_pnp_calibrator"
)
class TagBasedPNPCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.camera_optical_link_frame = kwargs["camera_optical_link_frame"]
        self.lidar_frame = kwargs["lidar_frame"]

        self.required_frames.append(self.camera_optical_link_frame)
        self.required_frames.append(self.lidar_frame)

        print("DefaultProject_TagBasedPNPCalibrator")

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.camera_optical_link_frame, child=self.lidar_frame),
            ],
        )
