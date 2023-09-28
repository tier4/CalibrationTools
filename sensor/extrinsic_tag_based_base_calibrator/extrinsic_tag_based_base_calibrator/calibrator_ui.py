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

from glob import glob
import os

from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget


class CalibratorUI(QMainWindow):
    def __init__(self, ros_interface):
        super().__init__()
        self.setWindowTitle("Tag based base-sensor calibration")

        # ROS Interface
        self.ros_interface = ros_interface

        self.num_valid_scenes = 0
        self.valid_scenes = False
        self.valid_current_scene_external_images = False
        self.valid_current_scene_calibration_images = False
        self.valid_current_scene_calibration_sensor_detections = False
        self.valid_external_camera_intrinsics = False
        self.processed_scenes = False
        self.database_loaded = False
        self.pending_service = False

        self.add_external_camera_images_to_scenes_service_status = False
        self.add_calibration_sensor_detections_to_new_scene_service_status = False
        self.load_external_camera_intrinsics_service_status = False
        self.save_external_camera_intrinsics_service_status = False
        self.calibrate_external_camera_intrinsics_service_status = False
        self.process_scenes_service_status = False
        self.calibration_service_status = False
        self.load_database_service_status = False
        self.save_database_service_status = False

        self.ros_interface.set_add_external_camera_images_to_scenes_callback(
            self.add_external_camera_images_to_scenes_result_callback,
            self.add_external_camera_images_to_scenes_status_callback,
        )
        self.ros_interface.set_add_calibration_sensor_detections_to_new_scene_callback(
            self.add_calibration_sensor_detections_to_new_scene_result_callback,
            self.add_calibration_sensor_detections_to_new_scene_status_callback,
        )

        self.ros_interface.set_load_external_camera_intrinsics_callback(
            self.load_external_camera_intrinsics_result_callback,
            self.load_external_camera_intrinsics_status_callback,
        )
        self.ros_interface.set_save_external_camera_intrinsics_callback(
            self.save_external_camera_intrinsics_result_callback,
            self.save_external_camera_intrinsics_status_callback,
        )
        self.ros_interface.set_calibrate_external_camera_intrinsics_callback(
            self.calibrate_external_camera_intrinsics_result_callback,
            self.calibrate_external_camera_intrinsics_status_callback,
        )
        self.ros_interface.set_process_scenes_callback(
            self.process_scenes_result_callback, self.process_scenes_status_callback
        )
        self.ros_interface.set_calibration_callback(
            self.calibration_result_callback, self.calibration_status_callback
        )
        self.ros_interface.set_load_database_callback(
            self.load_database_result_callback, self.load_database_status_callback
        )
        self.ros_interface.set_save_database_callback(
            self.save_database_result_callback, self.save_database_status_callback
        )

        self.widget = QWidget(self)
        self.setCentralWidget(self.widget)

        self.widget.setFixedWidth(400)
        self.layout = QVBoxLayout(self.widget)

        self.make_scene_group()
        self.make_intrinsics_group()
        self.make_calibration_group()
        self.make_database_group()

        self.layout.addWidget(self.scene_group)
        self.layout.addWidget(self.intrinsics_group)
        self.layout.addWidget(self.calibration_group)
        self.layout.addWidget(self.database_group)

        self.show()

    def make_scene_group(self):
        self.scene_group = QGroupBox("Scene group")
        self.scene_layout = QVBoxLayout()

        self.add_detections_button = QPushButton("Add detections to scene")
        self.add_detections_button.setEnabled(False)
        self.add_detections_button.clicked.connect(self.add_detections_button_callback)
        self.scene_layout.addWidget(self.add_detections_button)

        self.add_external_images_button = QPushButton("Add external camera images to scene")
        self.add_external_images_button.setEnabled(False)
        self.add_external_images_button.clicked.connect(self.add_external_images_button_callback)
        self.scene_layout.addWidget(self.add_external_images_button)

        self.scene_group.setLayout(self.scene_layout)

    def make_intrinsics_group(self):
        self.intrinsics_group = QGroupBox("Intrinsics")
        self.intrinsics_layout = QVBoxLayout()

        self.load_external_intrinsics_button = QPushButton("Load external camera intrinsics")
        self.load_external_intrinsics_button.setEnabled(False)
        self.load_external_intrinsics_button.clicked.connect(
            self.load_external_intrinsics_button_callback
        )
        self.intrinsics_layout.addWidget(self.load_external_intrinsics_button)

        self.save_external_intrinsics_button = QPushButton("Save external camera intrinsics")
        self.save_external_intrinsics_button.setEnabled(False)
        self.save_external_intrinsics_button.clicked.connect(
            self.save_external_intrinsics_button_callback
        )
        self.intrinsics_layout.addWidget(self.save_external_intrinsics_button)

        self.calibrate_external_intrinsics_button = QPushButton(
            "Calibrate external camera intrinsics"
        )
        self.calibrate_external_intrinsics_button.setEnabled(False)
        self.calibrate_external_intrinsics_button.clicked.connect(
            self.calibrate_external_intrinsics_button_callback
        )
        self.intrinsics_layout.addWidget(self.calibrate_external_intrinsics_button)

        self.intrinsics_group.setLayout(self.intrinsics_layout)

    def make_calibration_group(self):
        self.calibration_group = QGroupBox("Calibration")
        self.calibration_layout = QVBoxLayout()

        self.process_scenes_button = QPushButton("Process scenes")
        self.process_scenes_button.setEnabled(False)
        self.process_scenes_button.clicked.connect(self.process_scenes_button_callback)
        self.calibration_layout.addWidget(self.process_scenes_button)

        self.calibration_button = QPushButton("Calibrate base_link")
        self.calibration_button.setEnabled(False)
        self.calibration_button.clicked.connect(self.calibration_button_callback)
        self.calibration_layout.addWidget(self.calibration_button)

        self.calibration_group.setLayout(self.calibration_layout)

    def make_database_group(self):
        self.database_group = QGroupBox("Database")
        self.database_layout = QVBoxLayout()

        self.load_database_button = QPushButton("Load database")
        self.load_database_button.setEnabled(False)
        self.load_database_button.clicked.connect(self.load_database_button_callback)
        self.database_layout.addWidget(self.load_database_button)

        self.save_database_button = QPushButton("Save database")
        self.save_database_button.setEnabled(False)
        self.save_database_button.clicked.connect(self.save_database_button_callback)
        self.database_layout.addWidget(self.save_database_button)

        self.database_group.setLayout(self.database_layout)

    def check_status(self):
        self.valid_scenes = (
            self.valid_current_scene_external_images
            and self.valid_current_scene_calibration_sensor_detections
        )

        # Scene group
        self.add_external_images_button.setEnabled(
            self.num_valid_scenes > 0
            and self.add_external_camera_images_to_scenes_service_status
            and not self.pending_service
        )

        self.add_detections_button.setEnabled(
            self.add_calibration_sensor_detections_to_new_scene_service_status
            and not self.pending_service
        )

        # Intrinsics group
        self.load_external_intrinsics_button.setEnabled(
            self.load_external_camera_intrinsics_service_status
            and not self.valid_external_camera_intrinsics
            and not self.pending_service
        )
        self.save_external_intrinsics_button.setEnabled(
            self.save_external_camera_intrinsics_service_status
            and self.valid_external_camera_intrinsics
            and not self.pending_service
        )
        self.calibrate_external_intrinsics_button.setEnabled(
            self.calibrate_external_camera_intrinsics_service_status
            and not self.valid_external_camera_intrinsics
            and not self.pending_service
        )

        # Calibration group
        can_process = (
            self.valid_external_camera_intrinsics
            and self.num_valid_scenes > 0
            and not self.pending_service
        )
        self.process_scenes_button.setEnabled(self.process_scenes_service_status and can_process)
        self.calibration_button.setEnabled(
            self.calibration_service_status
            and (can_process and self.processed_scenes)
            or (self.database_loaded and self.valid_external_camera_intrinsics)
            and not self.pending_service
        )

        # Database group
        self.load_database_button.setEnabled(
            self.load_database_service_status
            and self.num_valid_scenes == 0
            and not self.processed_scenes
            and not self.pending_service
        )
        self.save_database_button.setEnabled(
            self.save_database_service_status and self.processed_scenes and not self.pending_service
        )

    def add_external_camera_images_to_scenes_result_callback(self, result):
        self.pending_service = False
        if result.success:
            self.valid_current_scene_external_images = True
        self.check_status()

    def add_external_camera_images_to_scenes_status_callback(self, status):
        self.add_external_camera_images_to_scenes_service_status = status
        self.check_status()

    def add_calibration_sensor_detections_to_new_scene_result_callback(self, result):
        self.pending_service = False
        if result.success:
            self.num_valid_scenes += 1
            self.valid_current_scene_calibration_sensor_detections = True
        self.check_status()

    def add_calibration_sensor_detections_to_new_scene_status_callback(self, status):
        self.add_calibration_sensor_detections_to_new_scene_service_status = status
        self.check_status()

    def load_external_camera_intrinsics_result_callback(self, result):
        self.pending_service = False
        self.valid_external_camera_intrinsics = result.success
        self.check_status()

    def load_external_camera_intrinsics_status_callback(self, status):
        self.load_external_camera_intrinsics_service_status = status
        self.check_status()

    def save_external_camera_intrinsics_result_callback(self, result):
        self.pending_service = False
        self.check_status()

    def save_external_camera_intrinsics_status_callback(self, status):
        self.save_external_camera_intrinsics_service_status = status
        self.check_status()

    def calibrate_external_camera_intrinsics_result_callback(self, result):
        self.pending_service = False
        self.valid_external_camera_intrinsics = result.success
        self.check_status()

    def calibrate_external_camera_intrinsics_status_callback(self, status):
        self.calibrate_external_camera_intrinsics_service_status = status
        self.check_status()

    def process_scenes_result_callback(self, result):
        self.pending_service = False
        self.processed_scenes = result.success
        self.check_status()

    def process_scenes_status_callback(self, status):
        self.process_scenes_service_status = status
        self.check_status()

    def calibration_result_callback(self, result):
        self.pending_service = False
        self.check_status()

    def calibration_status_callback(self, status):
        self.calibration_service_status = status
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

    def add_external_images_button_callback(self):
        # Here simply select folder
        # Iterate folder looking for scene{i}
        # Call the same service

        output_folder = QFileDialog.getExistingDirectory(
            None,
            "Select directory to save the calibration result",
            ".",
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )

        if output_folder is None or output_folder == "":
            return

        def filter_function(x):
            return x.lower().endswith(suffixes)

        scenes_images_paths = []

        for scene_id in range(self.num_valid_scenes):
            folder = os.path.join(output_folder, f"scene{scene_id}")
            suffixes = ("png", "jpg", "jpeg")
            scene_image_paths = list(filter(filter_function, glob(os.path.join(folder, "*"))))
            scenes_images_paths.append(scene_image_paths)

        self.pending_service = True
        self.ros_interface.add_external_camera_images_to_scenes(scenes_images_paths)
        self.check_status()

    def add_detections_button_callback(self):
        self.ros_interface.add_calibration_sensor_detections_to_new_scene()

    def load_external_intrinsics_button_callback(self):
        filename, _ = QFileDialog.getOpenFileName(
            None, "Load external camera intrinsics", ".", "Intrinsics file (*.yaml *.cfg)"
        )

        if len(filename) == 0:
            return

        self.pending_service = True
        self.ros_interface.load_external_camera_intrinsics([filename])
        self.check_status()

    def save_external_intrinsics_button_callback(self):
        filename, _ = QFileDialog.getSaveFileName(
            None, "Save extrinsics", ".", "Extrinsics file (*.yaml)"
        )

        if len(filename) == 0:
            return

        self.pending_service = True
        self.ros_interface.save_external_camera_intrinsics([filename])
        self.check_status()

    def calibrate_external_intrinsics_button_callback(self):
        filenames, _ = QFileDialog.getOpenFileNames(
            None, "Load external camera images", ".", "Image file (*.jpg *.JPG *.png *.PNG)"
        )

        if len(filenames) == 0:
            return

        self.pending_service = True
        self.ros_interface.calibrate_external_camera_intrinsics(filenames)
        self.check_status()

    def process_scenes_button_callback(self):
        self.pending_service = True
        self.ros_interface.process_scenes()
        self.check_status()

    def calibration_button_callback(self):
        self.pending_service = True
        self.ros_interface.calibrate()
        self.check_status()

    def load_database_button_callback(self):
        filename, _ = QFileDialog.getOpenFileName(None, "Open File", ".", "Database (*.db *.data)")

        if len(filename) == 0:
            return

        self.pending_service = True
        self.ros_interface.load_database([filename])
        self.check_status()

    def save_database_button_callback(self):
        filename, _ = QFileDialog.getSaveFileName(None, "Save File", ".", "Database (*.db *.data)")

        if len(filename) == 0:
            return

        self.pending_service = True
        self.ros_interface.save_database([filename])
        self.check_status()
