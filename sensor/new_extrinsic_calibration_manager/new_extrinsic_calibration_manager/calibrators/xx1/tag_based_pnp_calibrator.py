from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="xx1", calibrator_name="tag_based_pnp_calibrator"
)
class TagBasedPNPCalibrator(CalibratorBase):
    required_frames = ["sensor_kit_base_link", "velodyne_top_base_link", "velodyne_top"]

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.camera_name = kwargs["camera_name"]
        self.required_frames.append(f"{self.camera_name}/camera_link")
        self.required_frames.append(f"{self.camera_name}/camera_optical_link")

        print("TagBasedPNPCalibrator")
        print(self.camera_name, flush=True)

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent=f"{self.camera_name}/camera_optical_link", child="velodyne_top"),
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        camera_to_lidar_transform = calibration_transforms[
            f"{self.camera_name}/camera_optical_link"
        ]["velodyne_top"]
        sensor_kit_to_lidar_transform = self.get_transform_matrix(
            "sensor_kit_base_link", "velodyne_top"
        )
        camera_to_optical_link_transform = self.get_transform_matrix(
            f"{self.camera_name}/camera_link", f"{self.camera_name}/camera_optical_link"
        )
        sensor_kit_camera_link_transform = np.linalg.inv(
            camera_to_optical_link_transform
            @ camera_to_lidar_transform
            @ np.linalg.inv(sensor_kit_to_lidar_transform)
        )

        result = {
            "sensor_kit_base_link": {
                f"{self.camera_name}/camera_link": sensor_kit_camera_link_transform
            }
        }
        return result
