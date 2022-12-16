import threading
from typing import Dict
from typing import Optional
from typing import Tuple

from PySide2.QtCore import QObject
from PySide2.QtCore import Signal
from intrinsic_camera_calibrator.boards import BoardParameters
from intrinsic_camera_calibrator.parameter import ParameteredClass


class BoardDetector(ParameteredClass, QObject):

    detection_results_signal = Signal(object, object)

    def __init__(
        self, lock: threading.RLock, board_parameters: BoardParameters, cfg: Optional[Dict] = {}
    ):
        ParameteredClass.__init__(self, lock)
        QObject.__init__(self, None)
        self.board_parameters = board_parameters

        self.set_parameters(**cfg)

    def detect(self, img):
        raise NotImplementedError

    def single_shot_calibration_error(self, object_points, image_points) -> Tuple[float, float]:
        raise NotImplementedError
