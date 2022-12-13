from intrinsic_camera_calibrator.board_detectors.apriltag_grid_detector import ApriltagGridDetector
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.board_detectors.chessboard_detector import ChessBoardDetector
from intrinsic_camera_calibrator.board_detectors.dotboard_detector import DotBoardDetector
from intrinsic_camera_calibrator.boards import BoardEnum


def make_detector(board_type: BoardEnum, **kwargs) -> BoardDetector:
    classes_dic = {
        BoardEnum.CHESSBOARD: ChessBoardDetector,
        BoardEnum.DOTBOARD: DotBoardDetector,
        BoardEnum.APRILTAGGRID: ApriltagGridDetector,
    }
    return classes_dic[board_type](**kwargs)
