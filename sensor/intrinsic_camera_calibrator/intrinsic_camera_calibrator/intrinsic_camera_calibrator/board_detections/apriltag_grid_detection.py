from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection


class ApriltagGridDetection:
    def get_object_points(self):
        raise NotImplementedError

    def get_image_points(self):
        raise NotImplementedError

    def get_linear_error(self):
        raise NotImplementedError

    def get_rectified_linear_error(self):
        raise NotImplementedError

    def get_skew(self):
        raise NotImplementedError

    def get_size(self):
        raise NotImplementedError

    def get_speed(self, last: "BoardDetection"):
        raise NotImplementedError
