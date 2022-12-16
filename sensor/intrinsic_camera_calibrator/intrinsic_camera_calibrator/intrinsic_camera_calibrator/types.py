from enum import Enum


class OperationMode(Enum):
    IDLE = 0
    CALIBRATION = 1
    EVALUATION = 2


class ImageViewMode(Enum):
    SOURCE_UNRECTIFIED = 0
    SOURCE_RECTIFIED = 1
    DB_UNRECTIFIED = 2
    DB_RECTIFIED = 3
