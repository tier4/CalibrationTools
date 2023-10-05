from collections import defaultdict
from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="tier4_dummy_project", calibrator_name="tier4_base_lidar_calibration"
)
class DummyBaseLidarCalibrator(CalibratorBase):
    required_frames = ["base_link", "sensor_kit_base_link", "velodyne_top"]

    def __init__(self, ros_interface: RosInterface):
        super().__init__(ros_interface)

        print("Tier4DummyBaseLidarCalibrator")

        self.add_calibrator(
            service_name="calibrate_base_lidar",
            expected_calibration_frames=[FramePair(parent="base_link", child="velodyne_top")],
        )

    def post_process(self, calibration_transforms) -> Dict[str, Dict[str, np.array]]:
        sensor_kit_to_lidar = self.get_transform_matrix(
            parent="sensor_kit_base_link", child="velodyne_top"
        )

        base_link_to_sensor_kit = calibration_transforms["base_link"][
            "velodyne_top"
        ] @ np.linalg.inv(sensor_kit_to_lidar)

        output_transforms = defaultdict(lambda: defaultdict(np.array))
        output_transforms["base_link"]["sensor_kit_base_link"] = base_link_to_sensor_kit

        return output_transforms
