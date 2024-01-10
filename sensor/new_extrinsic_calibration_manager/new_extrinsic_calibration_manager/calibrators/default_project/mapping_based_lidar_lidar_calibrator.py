from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="default_project", calibrator_name="mapping_based_lidar_lidar_calibrator"
)
class MappingBasedLidarLidarCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.mapping_lidar_frame = kwargs["mapping_lidar_frame"]
        self.calibration_lidar_frames = kwargs["calibration_lidar_frames"]

        self.required_frames.extend([self.mapping_lidar_frame, *self.calibration_lidar_frames])

        print("default_MappingBasedLidarLidarCalibrator")

        self.add_calibrator(
            service_name="calibrate_lidar_lidar",
            expected_calibration_frames=[
                FramePair(parent=self.mapping_lidar_frame, child=calibration_lidar_frame)
                for calibration_lidar_frame in self.calibration_lidar_frames
            ],
        )
