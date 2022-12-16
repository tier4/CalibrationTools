from intrinsic_camera_calibrator.calibrators.calibrator import Calibrator
from intrinsic_camera_calibrator.calibrators.calibrator import CalibratorEnum
from intrinsic_camera_calibrator.calibrators.ceres_calibrator import CeresCalibrator
from intrinsic_camera_calibrator.calibrators.opencv_calibrator import OpenCVCalibrator


def make_calibrator(calibrator_type: CalibratorEnum, **kwargs) -> Calibrator:
    classes_dic = {
        CalibratorEnum.OPENCV: OpenCVCalibrator,
        CalibratorEnum.CERES: CeresCalibrator,
    }
    return classes_dic[calibrator_type](**kwargs)
