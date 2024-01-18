from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="x2", calibrator_name="tag_based_pnp_calibrator"
)
class TagBasedPNPCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.camera_name = kwargs["camera_name"]
        self.lidar_frame = kwargs["lidar_frame"]

        camera_name_to_sensor_kit_frame = {
            "camera0": "top_unit_base_link",
            "camera1": "top_unit_base_link",
            "camera2": "top_unit_base_link",
            "camera3": "rear_unit_base_link",
            "camera4": "top_unit_base_link",
            "camera5": "top_unit_base_link",
            "camera6": "front_unit_base_link",
            "camera7": "top_unit_base_link",
        }

        self.sensor_kit_frame = camera_name_to_sensor_kit_frame[self.camera_name]

        self.required_frames.extend(
            [
                self.lidar_frame,
                self.sensor_kit_frame,
                f"{self.camera_name}/camera_link",
                f"{self.camera_name}/camera_optical_link",
            ]
        )

        print("X2::TagBasedPNPCalibrator")

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent=f"{self.camera_name}/camera_optical_link", child=self.lidar_frame),
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        camera_to_lidar_transform = calibration_transforms[
            f"{self.camera_name}/camera_optical_link"
        ][self.lidar_frame]

        print(f"camera_to_lidar_transform={camera_to_lidar_transform}", flush=True)

        sensor_kit_to_lidar_transform = self.get_transform_matrix(
            self.sensor_kit_frame, self.lidar_frame
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
            self.sensor_kit_frame: {
                f"{self.camera_name}/camera_link": sensor_kit_camera_link_transform
            }
        }
        return result
