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

from PySide2.QtCore import QObject
from PySide2.QtCore import QThread
from PySide2.QtCore import Signal
import cv2
from intrinsic_camera_calibrator.data_sources.data_source import DataSource


class ImageFilesDataSource(DataSource, QObject):
    """Class that implements the DataSource to produce samples from a rosbag."""

    rosbag_topics_signal = Signal(object)
    consumed_signal = Signal()

    def __init__(self):
        DataSource.__init__(self)
        QObject.__init__(self, None)

        self.loop_images = False
        self.image_files_path = []
        self.image_index = 0
        self.paused = False

        self.consumed_signal.connect(self.on_consumed)

    def set_image_files(self, image_paths):
        """Set the images to use for calibration."""
        self.image_files_path = image_paths

    def start(self, loop: bool):

        self.loop_images = loop

        if len(self.image_files_path):
            self.send_data(self.image_files_path[self.image_index])
            self.image_index += 1

        self.thread = QThread()
        self.thread.start()
        self.moveToThread(self.thread)

    def consumed(self):
        """Send signal to the consumer having consumed an image. This method is executed in another thread, to a signal it is used to decouple."""
        self.consumed_signal.emit()

    def on_consumed(self):
        """Acts on the consumer having consumed an image. This method is executed in he source thread as it is connected to a local signal."""
        if self.image_index == len(self.image_files_path) and not self.loop_images:
            print("Produced all images!")
            return

        if self.paused:
            return

        self.image_index = self.image_index % len(self.image_files_path)
        self.send_data(self.image_files_path[self.image_index])
        self.image_index += 1

    def send_data(self, image_path):
        """Send a image message to the consumer prior transformation to a numpy array."""
        image = cv2.imread(image_path)
        self.data_callback(image)
