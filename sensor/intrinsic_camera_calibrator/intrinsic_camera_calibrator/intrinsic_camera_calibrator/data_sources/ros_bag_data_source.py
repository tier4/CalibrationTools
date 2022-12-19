#!/usr/bin/env python3

# Copyright 2022 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

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


def get_rosbag_options(path, serialization_format="cdr"):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3")

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    return storage_options, converter_options


class RosBagDataSource(DataSource, QObject):
    """Class that imlements the DataSource to produce samples from a rosbag."""

    rosbag_topics_signal = Signal(object)
    consumed_signal = Signal()

    def __init__(self):
        DataSource.__init__(self)
        QObject.__init__(self, None)

        self.bridge = CvBridge()

        self.rosbag_path = None
        self.consumed_signal.connect(self.on_consumed)

    def set_rosbag_file(self, rosbag_path):
        """Set the rosbag file and sendss a signal with all the image topic names if possible. If the file is metadata.yaml it instead reads the folder."""
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

        self.thread = QThread()
        self.thread.start()
        self.moveToThread(self.thread)

    def start(self, topic_name):

        # Parse camera name
        topic_namespaces = topic_name.split("/")
        topic_namespaces.reverse()
        image_index = [i for i, s in enumerate(topic_namespaces) if "image" in s][0]
        self.camera_name = topic_namespaces[image_index + 1]

        input_storage_options, input_converter_options = get_rosbag_options(self.rosbag_path)

        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(input_storage_options, input_converter_options)

        topic_types = self.reader.get_all_topics_and_types()

        self.type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

        storage_filter = rosbag2_py.StorageFilter(topics=[topic_name])
        self.reader.set_filter(storage_filter)

        if self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            self.send_data(topic, data)

    def consumed(self):
        """Send signal to the consumer having consumed an image. This method is executed in another thread, to a signal it is used to decouple."""
        self.consumed_signal.emit()

    def on_consumed(self):
        """Acts on the consumer having consumed an image. This method is executed in he source thread as it is connected to a local signal."""
        if self.reader.has_next():
            (topic, data, t) = self.reader.read_next()
            self.send_data(topic, data)
        else:
            print("bag ended !", flush=True)

    def send_data(self, topic, data):
        """Send a image message to the consumer prior transformation to a numpy array."""
        msg_type = get_message(self.type_map[topic])
        msg = deserialize_message(data, msg_type)

        if isinstance(msg, Image):
            image_data = self.bridge.imgmsg_to_cv2(msg)
            self.data_callback(image_data)

        elif isinstance(msg, CompressedImage):
            image_data = np.frombuffer(msg.data, np.uint8)
            image_data = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            self.data_callback(image_data)
