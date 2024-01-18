from collections import defaultdict
from typing import Dict

from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair
import numpy as np


@CalibratorRegistry.register_calibrator(
    project_name="x2", calibrator_name="tag_based_sfm_base_lidars_cameras_calibrator"
)
class TagBasedSfmBaseLidarsCamerasCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.base_frame = kwargs["base_frame"]
        self.top_unit_frame = "top_unit_base_link"
        self.front_unit_frame = "front_unit_base_link"
        self.rear_unit_frame = "rear_unit_base_link"

        self.main_sensor_frame = kwargs["main_calibration_sensor_frame"]
        self.calibration_lidar_frames = [
            kwargs["calibration_lidar_1_frame"],
            kwargs["calibration_lidar_2_frame"],
            kwargs["calibration_lidar_3_frame"],
        ]
        self.calibration_lidar_base_frames = [
            lidar_frame + "_base_link" for lidar_frame in self.calibration_lidar_frames
        ]

        self.calibration_camera_optical_link_frames = [
            kwargs["calibration_camera_0_frame"],
            kwargs["calibration_camera_1_frame"],
            kwargs["calibration_camera_2_frame"],
            kwargs["calibration_camera_3_frame"],
            kwargs["calibration_camera_4_frame"],
            kwargs["calibration_camera_5_frame"],
            kwargs["calibration_camera_6_frame"],
        ]
        self.calibration_camera_link_frames = [
            camera_frame.replace("camera_optical_link", "camera_link")
            for camera_frame in self.calibration_camera_optical_link_frames
        ]

        self.required_frames.extend(
            [
                self.base_frame,
                self.top_unit_frame,
                self.front_unit_frame,
                self.rear_unit_frame,
                self.main_sensor_frame,
                *self.calibration_lidar_frames,
                *self.calibration_lidar_base_frames,
                *self.calibration_camera_optical_link_frames,
                *self.calibration_camera_link_frames,
            ]
        )

        print("X2_tagBasedSfmBaseLidarsCamerasCalibrator")

        self.add_calibrator(
            service_name="calibrate_base_lidars_cameras",
            expected_calibration_frames=[
                FramePair(parent=self.main_sensor_frame, child=self.base_frame),
                *[
                    FramePair(parent=self.main_sensor_frame, child=calibration_frame)
                    for calibration_frame in self.calibration_lidar_frames
                ],
                *[
                    FramePair(parent=self.main_sensor_frame, child=calibration_frame)
                    for calibration_frame in self.calibration_camera_optical_link_frames
                ],
            ],
        )

    def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
        main_sensor_to_base_transform = calibration_transforms[self.main_sensor_frame][
            self.base_frame
        ]

        top_kit_to_main_lidar_transform = self.get_transform_matrix(
            self.top_unit_frame, self.main_sensor_frame
        )

        front_kit_to_front_lower_lidar_transform = self.get_transform_matrix(
            self.front_unit_frame, "pandar_40p_front"
        )

        rear_kit_to_rear_lower_lidar_transform = self.get_transform_matrix(
            self.rear_unit_frame, "pandar_40p_rear"
        )

        optical_link_to_camera_link_transforms = {
            camera_optical_link_frame: self.get_transform_matrix(
                camera_optical_link_frame, camera_link_frame
            )
            for camera_optical_link_frame, camera_link_frame in zip(
                self.calibration_camera_optical_link_frames, self.calibration_camera_link_frames
            )
        }

        base_to_top_kit_transform = np.linalg.inv(
            top_kit_to_main_lidar_transform @ main_sensor_to_base_transform
        )

        results = {self.base_frame: {}}

        base_to_front_kit_transform = (
            np.linalg.inv(main_sensor_to_base_transform)
            @ calibration_transforms[self.main_sensor_frame]["pandar_40p_front"]
            @ np.linalg.inv(front_kit_to_front_lower_lidar_transform)
        )
        base_to_rear_kit_transform = (
            np.linalg.inv(main_sensor_to_base_transform)
            @ calibration_transforms[self.main_sensor_frame]["pandar_40p_rear"]
            @ np.linalg.inv(rear_kit_to_rear_lower_lidar_transform)
        )

        results = defaultdict(dict)
        results[self.base_frame][self.top_unit_frame] = base_to_top_kit_transform
        results[self.base_frame][self.front_unit_frame] = base_to_front_kit_transform
        results[self.base_frame][self.rear_unit_frame] = base_to_rear_kit_transform

        top_cameras = ["camera0", "camera1", "camera2", "camera4", "camera5"]
        front_cameras = ["camera6"]
        rear_cameras = ["camera3"]

        for top_camera in top_cameras:
            results[self.top_unit_frame][f"{top_camera}/camera_link"] = (
                top_kit_to_main_lidar_transform
                @ calibration_transforms[self.main_sensor_frame][
                    f"{top_camera}/camera_optical_link"
                ]
                @ optical_link_to_camera_link_transforms[f"{top_camera}/camera_optical_link"]
            )

        for front_camera in front_cameras:
            results[self.front_unit_frame][f"{front_camera}/camera_link"] = (
                np.linalg.inv(base_to_front_kit_transform)
                @ np.linalg.inv(main_sensor_to_base_transform)
                @ calibration_transforms[self.main_sensor_frame][
                    f"{front_camera}/camera_optical_link"
                ]
                @ optical_link_to_camera_link_transforms[f"{front_camera}/camera_optical_link"]
            )

        for rear_camera in rear_cameras:
            results[self.rear_unit_frame][f"{rear_camera}/camera_link"] = (
                np.linalg.inv(base_to_rear_kit_transform)
                @ np.linalg.inv(main_sensor_to_base_transform)
                @ calibration_transforms[self.main_sensor_frame][
                    f"{rear_camera}/camera_optical_link"
                ]
                @ optical_link_to_camera_link_transforms[f"{rear_camera}/camera_optical_link"]
            )

        return results
