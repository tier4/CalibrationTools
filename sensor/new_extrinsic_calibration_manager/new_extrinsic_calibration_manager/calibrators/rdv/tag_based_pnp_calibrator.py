from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="rdv", calibrator_name="tag_based_pnp_calibrator"
)
class TagBasedPNPCalibrator(CalibratorBase):
    required_frames = ["sensor_kit_base_link", "pandar_top_base_link", "pandar_top"]

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.camera_name = kwargs["camera_name"]
        self.required_frames.append(f"{self.camera_name}/camera_link")
        self.required_frames.append(f"{self.camera_name}/camera_optical_link")

        print("RDV::TagBasedPNPCalibrator")
        print(self.camera_name, flush=True)

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent=f"{self.camera_name}/camera_optical_link", child="pandar_top"),
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        camera_to_lidar_transform = calibration_transforms[
            f"{self.camera_name}/camera_optical_link"
        ]["pandar_top"]

        print(f"camera_to_lidar_transform={camera_to_lidar_transform}", flush=True)

        sensor_kit_to_lidar_transform = self.get_transform_matrix(
            "sensor_kit_base_link", "pandar_top"
        )

        print(f"sensor_kit_to_lidar_transform={sensor_kit_to_lidar_transform}", flush=True)

        camera_to_optical_link_transform = self.get_transform_matrix(
            f"{self.camera_name}/camera_link", f"{self.camera_name}/camera_optical_link"
        )

        print(f"camera_to_optical_link_transform={camera_to_optical_link_transform}", flush=True)

        sensor_kit_camera_link_transform = np.linalg.inv(
            camera_to_optical_link_transform
            @ camera_to_lidar_transform
            @ np.linalg.inv(sensor_kit_to_lidar_transform)
        )

        print(f"sensor_kit_camera_link_transform={sensor_kit_camera_link_transform}", flush=True)

        result = {
            "sensor_kit_base_link": {
                f"{self.camera_name}/camera_link": sensor_kit_camera_link_transform
            }
        }
        return result
