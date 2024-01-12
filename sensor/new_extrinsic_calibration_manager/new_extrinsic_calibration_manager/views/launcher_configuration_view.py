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


from functools import reduce
from typing import Dict

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QGridLayout
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QLineEdit
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QScrollArea
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from ament_index_python.packages import get_package_share_directory
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.frontend import Parser
from launch.launch_description import LaunchDescription


class LauncherConfigurationView(QWidget):
    """A simple widget to visualize and edit a ParameterizedClass's parameters."""

    closed = Signal()
    launcher_parameters = Signal(dict)

    def __init__(self, project_name, calibrator_name):
        super().__init__()

        self.setWindowTitle("Launcher configuration")

        self.outer_layout = QVBoxLayout()
        self.inner_layout = QVBoxLayout()

        self.required_argument_layout = QGridLayout()
        self.optional_argument_layout = QGridLayout()

        self.required_arguments_group = QGroupBox("Required arguments:")
        self.required_arguments_group.setFlat(True)

        self.optional_arguments_group = QGroupBox("Optional arguments:")
        self.optional_arguments_group.setFlat(True)

        self.optional_arguments_dict = {}
        self.required_arguments_dict = {}
        self.arguments_widgets_dict = {}

        package_share_directory = get_package_share_directory("new_extrinsic_calibration_manager")
        launcher_path = (
            package_share_directory
            + "/launch/"
            + project_name
            + "/"
            + calibrator_name
            + ".launch.xml"
        )

        print(f"Reading xml from: {launcher_path}")

        try:
            with open(launcher_path) as f:
                root_entity, parser = Parser.load(f)
        except Exception as e:
            print("Failed reading xml file. Either not-existent or invalid")
            raise e

        ld: LaunchDescription = parser.parse_description(root_entity)

        for e in ld.entities:
            if not isinstance(e, DeclareLaunchArgument):
                continue

            description = e.description if e.description != "no description given" else ""

            if len(e.default_value) > 0:
                default_value = e.default_value[-1].text.replace(
                    " ", ""
                )  # KL: not sure if should the first or last default value

                self.optional_arguments_dict[e.name] = {
                    "value": default_value,
                    "description": description,
                    "choices": e.choices,
                }
            else:
                self.required_arguments_dict[e.name] = {
                    "value": "",
                    "description": description,
                    "choices": e.choices,
                }

        self.required_argument_layout.addWidget(QLabel("Name"), 0, 0)
        self.required_argument_layout.addWidget(QLabel("Value"), 0, 1)
        self.required_argument_layout.addWidget(QLabel("Help"), 0, 2)

        for i, (argument_name, argument_data) in enumerate(self.required_arguments_dict.items()):
            name_label = QLabel(argument_name)
            name_label.setMaximumWidth(400)

            default_value = argument_data["value"].replace(" ", "")

            if argument_data["choices"] is None or len(argument_data["choices"]) == 0:
                self.arguments_widgets_dict[argument_name] = QLineEdit(default_value)
                self.arguments_widgets_dict[argument_name].textChanged.connect(
                    self.check_argument_status
                )

            else:
                combo_box = QComboBox()

                for choice in argument_data["choices"]:
                    combo_box.addItem(choice)

                combo_box.currentTextChanged.connect(self.check_argument_status)
                self.arguments_widgets_dict[argument_name] = combo_box

            self.arguments_widgets_dict[argument_name].setMinimumWidth(400)
            self.arguments_widgets_dict[argument_name].setMaximumWidth(800)

            description_label = QLabel(argument_data["description"])
            description_label.setMaximumWidth(400)
            description_label.setToolTip(argument_data["description"])
            description_label.setText(argument_data["description"])

            self.required_argument_layout.addWidget(name_label, i + 1, 0)
            self.required_argument_layout.addWidget(
                self.arguments_widgets_dict[argument_name], i + 1, 1
            )
            self.required_argument_layout.addWidget(description_label, i + 1, 2)

        self.optional_argument_layout.addWidget(QLabel("Name"), 0, 0)
        self.optional_argument_layout.addWidget(QLabel("Value"), 0, 1)
        self.optional_argument_layout.addWidget(QLabel("Help"), 0, 2)

        for i, (argument_name, argument_data) in enumerate(self.optional_arguments_dict.items()):
            name_label = QLabel(argument_name)
            name_label.setMaximumWidth(400)

            if argument_data["choices"] is None or len(argument_data["choices"]) == 0:
                self.arguments_widgets_dict[argument_name] = QLineEdit(argument_data["value"])
                self.arguments_widgets_dict[argument_name].textChanged.connect(
                    self.check_argument_status
                )

            else:
                combo_box = QComboBox()

                for choice in argument_data["choices"]:
                    combo_box.addItem(choice)

                combo_box.currentTextChanged.connect(self.check_argument_status)
                self.arguments_widgets_dict[argument_name] = combo_box

            self.arguments_widgets_dict[argument_name].setMinimumWidth(400)
            self.arguments_widgets_dict[argument_name].setMaximumWidth(800)

            description_label = QLabel(argument_data["description"])
            description_label.setMaximumWidth(400)
            description_label.setToolTip(argument_data["description"])
            description_label.setText(argument_data["description"])

            self.optional_argument_layout.addWidget(name_label, i + 1, 0)
            self.optional_argument_layout.addWidget(
                self.arguments_widgets_dict[argument_name], i + 1, 1
            )
            self.optional_argument_layout.addWidget(description_label, i + 1, 2)

        self.required_arguments_group.setLayout(self.required_argument_layout)
        self.optional_arguments_group.setLayout(self.optional_argument_layout)

        self.launch_button = QPushButton("Launch")
        self.launch_button.clicked.connect(self.on_click)

        self.inner_layout.addWidget(self.required_arguments_group)
        self.inner_layout.addWidget(self.optional_arguments_group)
        self.inner_layout.addWidget(self.launch_button)

        self.widget = QWidget()
        self.widget.setLayout(self.inner_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidget(self.widget)

        self.outer_layout.addWidget(scroll_area)
        self.setLayout(self.outer_layout)

        self.check_argument_status()
        self.setMinimumWidth(self.widget.width() + 50)
        self.show()

    def check_argument_status(self, text=None):
        self.launch_button.setEnabled(
            reduce(
                lambda a, b: a and b,
                [
                    len(widget.text()) > 0 if hasattr(widget, "text") else widget.currentText()
                    for widget in self.arguments_widgets_dict.values()
                ],
            )
        )
        print("check_argument_status", flush=True)

    def on_click(self):
        args_dict: Dict[str, str] = {
            arg_name: args_widget.text()
            if hasattr(args_widget, "text")
            else args_widget.currentText()
            for arg_name, args_widget in self.arguments_widgets_dict.items()
        }

        def is_list(arg: str):
            return len(arg) >= 2 and arg[0] == "[" and arg[-1] == "]"

        for key, value in args_dict.items():
            if is_list(value):
                args_dict[key] = [item.strip() for item in value.strip("[]").split(",")]

        print(args_dict, flush=True)

        self.launcher_parameters.emit(args_dict)
        self.close()

    def closeEvent(self, event):
        """When the widget is closed it should be marked for deletion and notify the event."""
        self.closed.emit()
        event.accept()
        self.deleteLater()
