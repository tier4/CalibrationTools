from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="dummy_project", calibrator_name="camera_lidar_calibration"
)
class DummyCameraLidarCalibrator(CalibratorBase):
    required_frames = [
        "sensor_kit_base_link",
        "camera0/camera_link",
        "camera0/camera_optical_link",
        "velodyne_top_base_link",
        "velodyne_top",
    ]

    def __init__(self, ros_interface: RosInterface):
        super().__init__(ros_interface)

        print("DummyCameraLidarCalibrator")

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent="velodyne_top", child="camera0/camera_link"),
            ],
        )
