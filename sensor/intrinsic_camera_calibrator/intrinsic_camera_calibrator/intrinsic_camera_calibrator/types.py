from enum import Enum


class OperationMode(Enum):
    IDLE = 0
    CALIBRATION = 1
    EVALUATION = 2


class ImageViewMode(Enum):
    SOURCE_UNRECTIFIED = "Source unrectified"
    SOURCE_RECTIFIED = "Source rectified"
    TRAINING_DB_UNRECTIFIED = "Training DB unrectified"
    EVALUATION_DB_UNRECTIFIED = "Evaluation DB unrectified"
