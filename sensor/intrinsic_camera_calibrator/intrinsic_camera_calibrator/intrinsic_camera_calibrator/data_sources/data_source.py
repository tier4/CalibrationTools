from enum import Enum
import threading


class DataSourceEnum(Enum):
    TOPIC = "ROS topic"
    BAG2 = "ROS bag"
    VIDEO = "Video file"
    FILES = "Image files"

    def __str__(self):
        return str(self.value)


class DataSource:
    def __init__(self, **kwargs):
        self.lock = threading.RLock()
        self.data_callback = None
        self.pending_data_not_consumed = False
        pass

    def set_data_callback(self, callback):
        self.data_callback = callback
        self.pending_data_not_consumed = True

    def consumed(self):
        with self.lock:
            self.pending_data_not_consumed = False
