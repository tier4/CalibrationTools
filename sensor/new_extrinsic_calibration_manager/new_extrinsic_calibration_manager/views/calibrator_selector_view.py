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

import logging

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry


class CalibrationSelectorView(QWidget):
    """Initial widget to let the user configure the calibrator."""

    selected_calibrator_signal = Signal(str, str)

    def __init__(self):
        super().__init__()

        self.setWindowTitle("Calibrator selection menu")
        self.setMinimumWidth(300)

        self.layout = QVBoxLayout(self)

        # calibrator
        self.calibrator_group = QGroupBox("Calibrator:")
        self.calibrator_group.setFlat(True)

        self.calibrator_combobox = QComboBox()

        calibrator_layout = QVBoxLayout()
        calibrator_layout.addWidget(self.calibrator_combobox)
        self.calibrator_group.setLayout(calibrator_layout)

        def onNewProjectName(new_project_name):
            self.calibrator_combobox.clear()

            for calibrator_name in CalibratorRegistry.getProjectCalibrators(new_project_name):
                self.calibrator_combobox.addItem(calibrator_name)

                # Project

        self.project_group = QGroupBox("Project:")
        self.project_group.setFlat(True)

        self.project_combobox = QComboBox()
        self.project_combobox.currentTextChanged.connect(onNewProjectName)

        for project_name in CalibratorRegistry.getProjects():
            self.project_combobox.addItem(project_name)

        project_layout = QVBoxLayout()
        project_layout.addWidget(self.project_combobox)
        self.project_group.setLayout(project_layout)

        self.start_button = QPushButton("Continue")
        self.start_button.clicked.connect(self.on_click)

        self.layout.addWidget(self.project_group)
        self.layout.addWidget(self.calibrator_group)
        self.layout.addWidget(self.start_button)

        self.show()

    def on_click(self):
        """Start the calibration process after receiving the user settings."""
        self.selected_calibrator_signal.emit(
            self.project_combobox.currentText(), self.calibrator_combobox.currentText()
        )
        self.close()

    def closeEvent(self, event):
        """When the window is closed, the widget is marked for deletion."""
        # self.closed.emit()
        event.accept()
        self.deleteLater()
        logging.info("Closing calibrator selector view")
