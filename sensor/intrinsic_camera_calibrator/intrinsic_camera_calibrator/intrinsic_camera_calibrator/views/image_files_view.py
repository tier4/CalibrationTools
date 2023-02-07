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


import os

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QCheckBox
from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from intrinsic_camera_calibrator.data_sources.image_files_data_source import ImageFilesDataSource


class ImageFilesView(QWidget):
    """Widget to configure and initialize a RosBagDataSource."""

    failed = Signal()
    success = Signal()

    set_image_files_request = Signal(object)
    start_request = Signal(bool)

    def __init__(self, data_source: ImageFilesDataSource):
        self.data_source = data_source

        super().__init__()

        self.setWindowTitle("Select multiple images to calibrate")
        self.setMinimumWidth(300)

        self.images_selected = False

        self.data_source = data_source
        self.set_image_files_request.connect(self.data_source.set_image_files)
        self.start_request.connect(self.data_source.start)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.select_bag_button = QPushButton("Select image files")
        self.layout.addWidget(self.select_bag_button)

        self.loop_images_checkbox = QCheckBox("Loop images")
        self.loop_images_checkbox.setChecked(False)

        self.accept_button = QPushButton("Ok")
        self.accept_button.setEnabled(False)
        self.layout.addWidget(self.loop_images_checkbox)
        self.layout.addWidget(self.accept_button)

        self.select_bag_button.clicked.connect(self.select_bag_callback)
        self.accept_button.clicked.connect(self.accept_callback)

        self.show()

    def select_bag_callback(self):
        """Create a blocking dialog to select a rosbag."""
        files = QFileDialog.getOpenFileNames(
            self,
            "Select calibration images",
            os.getcwd(),
            "files (*.png *.bmp *.jpg *.jpeg *.JPG *.JPEG)",
        )[0]

        if len(files) == 0:
            self.failed.emit()
            return

        self.images_selected = False
        self.accept_button.setEnabled(True)
        self.set_image_files_request.emit(files)

    def accept_callback(self):
        """Initialize the data source."""
        self.success.emit()
        self.start_request.emit(self.loop_images_checkbox.isChecked())
        self.close()

    def closeEvent(self, event):
        """When the widget is closed it should be marked for deletion and notify the event."""
        if not self.images_selected:
            self.failed.emit()
        event.accept()
        self.deleteLater()
