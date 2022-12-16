import threading
from typing import Dict
from typing import List

from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.calibrators.calibrator import Calibrator
from intrinsic_camera_calibrator.camera_model import CameraModel
from intrinsic_camera_calibrator.parameter import Parameter


class CeresCalibrator(Calibrator):
    def __init__(self, lock: threading.RLock, cfg: Dict = {}):
        super().__init__(lock, cfg)

        print("Ceres calibrator constructor")

        self.some_parameter_name = Parameter(int, value=2, min_value=0, max_value=6)
        self.set_parameters(**cfg)

    def _calibration_impl(self, detections: List[BoardDetection]) -> CameraModel:

        raise NotImplementedError
