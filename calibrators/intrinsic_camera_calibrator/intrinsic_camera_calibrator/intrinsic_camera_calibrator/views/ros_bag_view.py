#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc.
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

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from intrinsic_camera_calibrator.data_sources.ros_bag_data_source import RosBagDataSource


class RosBagView(QWidget):
    """Widget to configure and initialize a RosBagDataSource."""

    failed = Signal()
    success = Signal()

    set_rosbag_request = Signal(str)
    rosbag_start_request = Signal()

    def __init__(self, data_source: RosBagDataSource):
        self.data_source = data_source

        super().__init__()

        self.setWindowTitle("Select a ros bag and topic")
        self.setMinimumWidth(300)

        self.bag_selected = False
        self.topic_selected = False

        self.data_source = data_source
        self.set_rosbag_request.connect(self.data_source.set_rosbag_file)
        self.data_source.rosbag_topics_signal.connect(self.update_topics)
        self.rosbag_start_request.connect(self.data_source.start)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.select_bag_button = QPushButton("Select ROS bag")
        self.layout.addWidget(self.select_bag_button)

        self.label = QLabel("Select topic")
        self.layout.addWidget(self.label)

        self.topic_combo_box = QComboBox()
        self.topic_combo_box.setEnabled(False)

        def on_text_changed(image_topic):
            image_topic = self.topic_combo_box.currentText()
            self.data_source.set_topic(image_topic)
            self.topic_selected = True

        self.topic_combo_box.currentTextChanged.connect(on_text_changed)

        self.layout.addWidget(self.topic_combo_box)

        self.accept_button = QPushButton("Ok")
        self.accept_button.setEnabled(False)
        self.layout.addWidget(self.accept_button)

        self.select_bag_button.clicked.connect(self.select_bag_callback)
        self.accept_button.clicked.connect(self.accept_callback)

        self.topic_combo_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.show()

    def select_bag_callback(self):
        """Create a blocking dialog to select a rosbag."""
        fname = QFileDialog.getOpenFileName()
        self.set_rosbag_request.emit(fname[0])

    def update_topics(self, topic_list):
        """Update the topics combobox and adjust the size of the widget based on a list of topics."""
        topic_list.sort()

        for topic in topic_list:
            self.topic_combo_box.addItem(topic)

        if len(topic_list):
            self.topic_combo_box.setEnabled(True)
            self.accept_button.setEnabled(True)

        self.adjustSize()

    def accept_callback(self):
        """Process the user choice of topic."""
        if self.topic_combo_box.count() == 0:
            return

        self.success.emit()
        self.rosbag_start_request.emit()
        self.close()

    def closeEvent(self, event):
        """When the widget is closed it should be marked for deletion and notify the event."""
        if not self.topic_selected:
            self.failed.emit()
        event.accept()

        self.deleteLater()
