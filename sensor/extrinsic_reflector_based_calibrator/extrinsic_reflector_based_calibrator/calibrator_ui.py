#!/usr/bin/env python3

# Copyright 2023 Tier IV, Inc.
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

from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget


class CalibratorUI(QMainWindow):
    def __init__(self, ros_interface):
        super().__init__()
        self.setWindowTitle("Reflector-based lidar-radar calibrator")

        # ROS Interface
        self.ros_interface = ros_interface

        self.pending_service = False
        self.calibration_sent = False
        self.background_model_done = False

        self.extract_background_model_status = False
        self.add_lidar_radar_pair_status = False
        self.send_calibration_status = False

        self.ros_interface.set_extract_background_model_callback(
            self.extract_background_model_result_callback,
            self.extract_background_model_status_callback,
        )
        self.ros_interface.set_add_lidar_radar_pair_callback(
            self.add_lidar_radar_pair_result_callback,
            self.add_lidar_radar_pair_status_callback,
        )

        self.ros_interface.set_send_calibration_callback(
            self.send_calibration_result_callback,
            self.send_calibration_status_callback,
        )

        self.widget = QWidget(self)
        self.setCentralWidget(self.widget)

        self.widget.setFixedWidth(250)
        self.layout = QVBoxLayout(self.widget)

        self.extract_background_model_button = QPushButton("Extract background model")
        self.extract_background_model_button.setEnabled(False)
        self.extract_background_model_button.clicked.connect(
            self.extract_background_model_button_callback
        )
        self.layout.addWidget(self.extract_background_model_button)

        self.add_lidar_radar_pair_button = QPushButton("Add lidar-radar pair")
        self.add_lidar_radar_pair_button.setEnabled(False)
        self.add_lidar_radar_pair_button.clicked.connect(self.add_lidar_radar_pair_button_callback)
        self.layout.addWidget(self.add_lidar_radar_pair_button)

        self.send_calibration_button = QPushButton("Send calibration")
        self.send_calibration_button.setEnabled(False)
        self.send_calibration_button.clicked.connect(self.send_calibration_button_callback)
        self.layout.addWidget(self.send_calibration_button)

        self.show()

    def check_status(self):
        disable_buttons = self.calibration_sent or self.pending_service
        self.extract_background_model_button.setEnabled(
            not self.background_model_done
            and self.extract_background_model_status
            and not disable_buttons
        )
        self.add_lidar_radar_pair_button.setEnabled(
            self.add_lidar_radar_pair_status and not disable_buttons
        )
        self.send_calibration_button.setEnabled(
            self.send_calibration_status and not disable_buttons
        )

    def extract_background_model_result_callback(self, result):
        self.pending_service = False
        self.background_model_done = True
        self.check_status()

    def extract_background_model_status_callback(self, status):
        self.extract_background_model_status = status
        self.check_status()

    def add_lidar_radar_pair_result_callback(self, result):
        self.pending_service = False
        self.check_status()

    def add_lidar_radar_pair_status_callback(self, status):
        self.add_lidar_radar_pair_status = status
        self.check_status()

    def send_calibration_result_callback(self, result):
        self.pending_service = False
        self.calibration_sent = True
        self.check_status()

    def send_calibration_status_callback(self, status):
        self.send_calibration_status = status
        self.check_status()

    def extract_background_model_button_callback(self):
        self.pending_service = True
        self.ros_interface.extract_background_model()
        self.check_status()

    def add_lidar_radar_pair_button_callback(self):
        self.pending_service = True
        self.ros_interface.add_lidar_radar_pair()
        self.check_status()

    def send_calibration_button_callback(self):
        self.pending_service = True
        self.ros_interface.send_calibration()
        self.check_status()
