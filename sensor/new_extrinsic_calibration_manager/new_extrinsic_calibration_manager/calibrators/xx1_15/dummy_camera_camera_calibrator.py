from new_extrinsic_calibration_manager.calibrator_base import CalibratorBase
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="dummy_project", calibrator_name="camera_camera_calibration"
)
class DummyCameraCameraCalibrator(CalibratorBase):
    required_frames = [
        "sensor_kit_base_link",
        "camera0/camera_link",
        "camera0/camera_optical_link",
        "camera1/camera_optical_link",
        "camera1/camera_link",
    ]

    def __init__(self, ros_interface: RosInterface):
        super().__init__(ros_interface)

        print("DummyCameraCameraCalibrator")

        self.add_calibrator(
            service_name="calibrate_camera_camera",
            expected_calibration_frames=[
                FramePair(parent="camera0/camera_link", child="camera1/camera_link"),
            ],
        )
