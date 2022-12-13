from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.parameter import Parameter


class ApriltagGridDetector(BoardDetector):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.quad_decimate = Parameter(int, value=1, min=1, max=4)
        self.quad_sigma = Parameter(float, value=0.0, min=0.0, max=1.0)
        pass

    def detect(self, img):
        return super().detect(img)
