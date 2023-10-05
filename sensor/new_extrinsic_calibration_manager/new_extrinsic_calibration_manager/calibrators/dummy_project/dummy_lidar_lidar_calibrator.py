from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="dummy_project", calibrator_name="lidar_lidar_calibration"
)
class DummyLidarLidarCalibrator(CalibratorBase):
    required_frames = [
        "sensor_kit_base_link",
        "velodyne_top_base_link",
        "velodyne_top",
        "velodyne_left_base_link",
        "velodyne_left",
        "velodyne_right_base_link",
        "velodyne_right",
    ]

    def __init__(self, ros_interface: RosInterface):
        super().__init__(ros_interface)

        print("DummyLidarLidarCalibrator")

        self.add_calibrator(
            service_name="calibrate_camera_lidar",
            expected_calibration_frames=[
                FramePair(parent="velodyne_top", child="velodyne_left"),
                FramePair(parent="velodyne_top", child="velodyne_right"),
            ],
        )
