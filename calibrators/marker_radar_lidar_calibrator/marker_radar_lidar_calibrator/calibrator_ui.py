#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QLineEdit
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget


class CalibratorUI(QMainWindow):
    def __init__(self, ros_interface):
        super().__init__()
        self.setWindowTitle("Marker radar-lidar calibrator")

        # ROS Interface
        self.ros_interface = ros_interface

        self.pending_service = False
        self.calibration_sent = False
        self.background_model_done = False
        self.database_loaded = False

        self.extract_background_model_service_status = False
        self.add_lidar_radar_pair_service_status = False
        self.delete_lidar_radar_pair_service_status = False
        self.send_calibration_service_status = False
        self.load_database_service_status = False
        self.save_database_service_status = False

        self.ros_interface.set_extract_background_model_callback(
            self.extract_background_model_result_callback,
            self.extract_background_model_service_status_callback,
        )

        self.ros_interface.set_add_lidar_radar_pair_callback(
            self.add_lidar_radar_pair_result_callback,
            self.add_lidar_radar_pair_service_status_callback,
        )

        self.ros_interface.set_delete_lidar_radar_pair_callback(
            self.delete_lidar_radar_pair_result_callback,
            self.delete_lidar_radar_pair_service_status_callback,
        )

        self.ros_interface.set_send_calibration_callback(
            self.send_calibration_result_callback,
            self.send_calibration_service_status_callback,
        )

        self.ros_interface.set_load_database_callback(
            self.load_database_result_callback,
            self.load_database_status_callback,
        )

        self.ros_interface.set_save_database_callback(
            self.save_database_result_callback, self.save_database_status_callback
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

        self.delete_pair_label = QLabel("Enter ID to delete pair:")
        self.layout.addWidget(self.delete_pair_label)
        self.delete_pair_input = QLineEdit()
        self.delete_pair_input.setPlaceholderText("Enter pair ID")
        self.delete_pair_input.setText("-1")  # Set the default value as -1
        self.layout.addWidget(self.delete_pair_input)

        self.delete_lidar_radar_pair_button = QPushButton("Delete lidar-radar pair")
        self.delete_lidar_radar_pair_button.setEnabled(False)
        self.delete_lidar_radar_pair_button.clicked.connect(
            self.delete_lidar_radar_pair_button_callback
        )
        self.layout.addWidget(self.delete_lidar_radar_pair_button)

        self.send_calibration_button = QPushButton("Send calibration")
        self.send_calibration_button.setEnabled(False)
        self.send_calibration_button.clicked.connect(self.send_calibration_button_callback)
        self.layout.addWidget(self.send_calibration_button)

        self.load_database_button = QPushButton("Load database")
        self.load_database_button.setEnabled(False)
        self.load_database_button.clicked.connect(self.load_database_button_callback)
        self.layout.addWidget(self.load_database_button)

        self.save_database_button = QPushButton("Save database")
        self.save_database_button.setEnabled(False)
        self.save_database_button.clicked.connect(self.save_database_button_callback)
        self.layout.addWidget(self.save_database_button)

        self.show()

    def check_status(self):
        disable_buttons = self.calibration_sent or self.pending_service
        self.extract_background_model_button.setEnabled(
            not self.background_model_done
            and self.extract_background_model_service_status
            and not disable_buttons
        )
        self.add_lidar_radar_pair_button.setEnabled(
            self.add_lidar_radar_pair_service_status and not disable_buttons
        )
        self.delete_lidar_radar_pair_button.setEnabled(
            self.delete_lidar_radar_pair_service_status and not disable_buttons
        )
        self.send_calibration_button.setEnabled(
            self.send_calibration_service_status and not disable_buttons
        )

        self.load_database_button.setEnabled(
            self.load_database_service_status and not disable_buttons and not self.database_loaded
        )
        self.save_database_button.setEnabled(
            self.save_database_service_status and not disable_buttons
        )

    def extract_background_model_result_callback(self, result):
        self.pending_service = False
        self.background_model_done = True
        self.check_status()

    def extract_background_model_service_status_callback(self, status):
        self.extract_background_model_service_status = status
        self.check_status()

    def add_lidar_radar_pair_result_callback(self, result):
        self.pending_service = False
        self.check_status()

    def add_lidar_radar_pair_service_status_callback(self, status):
        self.add_lidar_radar_pair_service_status = status
        self.check_status()

    def delete_lidar_radar_pair_result_callback(self, result):
        self.pending_service = False
        self.check_status()

    def delete_lidar_radar_pair_service_status_callback(self, status):
        self.delete_lidar_radar_pair_service_status = status
        self.check_status()

    def send_calibration_result_callback(self, result):
        self.pending_service = False
        self.calibration_sent = True
        self.check_status()

    def send_calibration_service_status_callback(self, status):
        self.send_calibration_service_status = status
        self.check_status()

    def load_database_result_callback(self, result):
        self.pending_service = False
        self.database_loaded = result.success
        self.check_status()

    def load_database_status_callback(self, status):
        self.load_database_service_status = status
        self.check_status()

    def save_database_result_callback(self, result):
        self.pending_service = False
        self.check_status()

    def save_database_status_callback(self, status):
        self.save_database_service_status = status
        self.check_status()

    def extract_background_model_button_callback(self):
        self.pending_service = True
        self.ros_interface.extract_background_model()
        self.check_status()

    def add_lidar_radar_pair_button_callback(self):
        self.pending_service = True
        self.ros_interface.add_lidar_radar_pair()
        self.check_status()

    def delete_lidar_radar_pair_button_callback(self):
        pair_id_text = self.delete_pair_input.text()
        try:
            pair_id = int(pair_id_text)  # Convert input to integer
        except ValueError:
            print("Please enter a valid numeric pair ID.")
            return

        self.pending_service = True
        self.ros_interface.delete_lidar_radar_pair(pair_id)
        self.check_status()

    def send_calibration_button_callback(self):
        self.pending_service = True
        self.ros_interface.send_calibration()
        self.check_status()

    def load_database_button_callback(self):
        filename, _ = QFileDialog.getOpenFileName(None, "Open File", ".", "Text Files (*.txt)")

        if len(filename) == 0:
            return

        self.pending_service = True
        self.ros_interface.load_database(filename)
        self.check_status()

    def save_database_button_callback(self):
        filename, _ = QFileDialog.getSaveFileName(None, "Save File", ".", "Text Files (*.txt)")
        if len(filename) == 0:
            return

        if not filename.endswith(".txt"):
            filename += ".txt"

        self.pending_service = True
        self.ros_interface.save_database(filename)
        self.check_status()
