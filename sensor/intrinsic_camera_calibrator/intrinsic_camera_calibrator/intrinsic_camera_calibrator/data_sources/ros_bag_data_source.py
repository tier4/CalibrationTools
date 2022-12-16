import os
from pathlib import Path
import sys
import threading

from PySide2.QtCore import QObject
from PySide2.QtCore import QThread
from PySide2.QtCore import Signal
import cv2
from cv_bridge import CvBridge
from intrinsic_camera_calibrator.data_sources.data_source import DataSource
import numpy as np
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image

if os.environ.get("ROSBAG2_PY_TEST_WITH_RTLD_GLOBAL", None) is not None:
    # This is needed on Linux when compiling with clang/libc++.
    # TL;DR This makes class_loader work when using a python extension compiled with libc++.
    #
    # For the fun RTTI ABI details, see https://whatofhow.wordpress.com/2015/03/17/odr-rtti-dso/.
    sys.setdlopenflags(os.RTLD_GLOBAL | os.RTLD_LAZY)


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


class RosBagDataSource(DataSource, QObject):

    rosbag_topics_signal = Signal(object)
    consumed_signal = Signal()

    def __init__(self):
        DataSource.__init__(self)
        QObject.__init__(self, None)

        self.thread = QThread()
        self.thread.start()
        self.moveToThread(self.thread)

        self.bridge = CvBridge()
        self.lock = threading.RLock()

        self.rosbag_path = None
        self.consumed_signal.connect(self.on_consumed)

    def set_rosbag_file(self, rosbag_path):

        if Path(rosbag_path).name == "metadata.yaml":
            self.rosbag_path = str(Path(rosbag_path).parent)
        else:
            self.rosbag_path = rosbag_path

        input_storage_options, input_converter_options = get_rosbag_options(self.rosbag_path)

        reader = rosbag2_py.SequentialReader()
        reader.open(input_storage_options, input_converter_options)

        topic_types = reader.get_all_topics_and_types()

        # Create a map for quicker lookup
        self.type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        self.rosbag_topics_signal.emit(
            [
                topic_name
                for topic_name, topics_type in self.type_map.items()
                if topics_type == "sensor_msgs/msg/CompressedImage"
                or topics_type == "sensor_msgs/msg/Image"
            ]
        )

    def start(self, topic_name):

        input_storage_options, input_converter_options = get_rosbag_options(self.rosbag_path)

        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(input_storage_options, input_converter_options)

        topic_types = self.reader.get_all_topics_and_types()

        # Create a map for quicker lookup
        self.type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
        self.reader.set_filter(storage_filter)

        if self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            self.send_data(topic, data)

    def consumed(self):
        self.consumed_signal.emit()

    def on_consumed(self):

        if self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            self.send_data(topic, data)
        else:
            print("bag ended !", flush=True)

    def send_data(self, topic, data):

        msg_type = get_message(self.type_map[topic])
        msg = deserialize_message(data, msg_type)

        if isinstance(msg, Image):

            image_data = self.bridge.imgmsg_to_cv2(msg)
            self.data_callback(image_data)

        elif isinstance(msg, CompressedImage):

            image_data = np.frombuffer(msg.data, np.uint8)
            image_data = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            self.data_callback(image_data)
