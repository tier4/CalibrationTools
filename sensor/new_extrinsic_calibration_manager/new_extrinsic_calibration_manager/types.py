from enum import Enum
from typing import NamedTuple


class CalibratorState(Enum):
    WAITING_TFS = 1
    WAITING_SERVICES = 2
    READY = 3
    CALIBRATING = 4
    FINISHED = 5


class FramePair(NamedTuple):
    parent: str
    child: str
