from enum import Enum

from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.parameter import ParameteredClass


class BoardEnum(Enum):
    CHESSBOARD = {"name": "chess_board", "display": "Chess board"}
    DOTBOARD = {"name": "dot_board", "display": "Dot board"}
    APRILTAGGRID = {"name": "apriltag_grid", "display": "Apriltag grid"}

    def from_name(name: str) -> "BoardEnum":
        for board in BoardEnum:
            if name == board.value["name"]:
                return board

        raise ValueError

    def from_index(i: int):
        return list(BoardEnum)[i]

    def get_id(self) -> int:
        for index, board in enumerate(BoardEnum):
            if board == self:
                return index

        raise ValueError


class BoardParameters(ParameteredClass):
    def __init__(self, lock, cfg):

        self.rows = Parameter(int, value=1, min_value=1, max_value=20)
        self.cols = Parameter(int, value=1, min_value=1, max_value=20)
        self.cell_size = Parameter(float, value=0.1, min_value=0.01, max_value=1.0)

        super().__init__(lock, cfg)
