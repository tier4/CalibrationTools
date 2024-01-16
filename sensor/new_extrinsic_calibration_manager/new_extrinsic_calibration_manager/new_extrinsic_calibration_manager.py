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

from collections import defaultdict
import copy
from functools import partial
import os
import signal
import subprocess
import sys
import threading
from typing import Dict

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QApplication
from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QHBoxLayout
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QTableView
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.frontend import Parser
from launch.launch_description import LaunchDescription
from new_extrinsic_calibration_manager.calibration_manager_model import CalibratorManagerModel
from new_extrinsic_calibration_manager.calibrator_base import CalibratorState
from new_extrinsic_calibration_manager.calibrator_registry import CalibratorRegistry
import new_extrinsic_calibration_manager.calibrators  # noqa: F401 Force imports
from new_extrinsic_calibration_manager.ros_interface import RosInterface
from new_extrinsic_calibration_manager.views.calibrator_selector_view import CalibrationSelectorView
from new_extrinsic_calibration_manager.views.launcher_configuration_view import (
    LauncherConfigurationView,
)
from new_extrinsic_calibration_manager.views.tf_view import TfView
import rclpy


class NewExtrinsicCalibrationManager(QMainWindow):
    tfs_graph_signal = Signal(object)

    def __init__(self):
        super().__init__()

        self.central_widget = QWidget(self)
        # self.central_widget.resize(1000,1000)

        self.setCentralWidget(self.central_widget)
        # self.setWindowTitle("New extrinsic calibration manager")

        self.ros_interface: RosInterface = None
        self.tfs_dict: Dict[str, Dict[str, None]] = defaultdict(lambda: defaultdict(None))

        # Threading variables
        self.lock = threading.RLock()

        # Control widgets
        self.action_button = QPushButton("Calibrate")
        self.action_button.setEnabled(False)
        self.status_label = QLabel("Not ready")

        # MVC
        self.calibrators_view = QTableView()

        # Tf Views
        self.initial_tfs_view = TfView()
        self.calibration_tfs_view = TfView()
        self.final_tfs_view = TfView()

        self.initial_tfs_group = QGroupBox("Initial TF tree")
        initial_tfs_layout = QVBoxLayout()
        initial_tfs_layout.addWidget(self.initial_tfs_view)
        self.initial_tfs_group.setLayout(initial_tfs_layout)

        self.calibration_tfs_group = QGroupBox("Calibration tree")
        calibration_tfs_layout = QVBoxLayout()
        calibration_tfs_layout.addWidget(self.calibration_tfs_view)
        self.calibration_tfs_group.setLayout(calibration_tfs_layout)

        self.final_tfs_group = QGroupBox("Final TF tree")
        final_tfs_layout = QVBoxLayout()
        final_tfs_layout.addWidget(self.final_tfs_view)
        self.final_tfs_group.setLayout(final_tfs_layout)

        self.main_layout = QHBoxLayout(self.central_widget)

        self.control_layout = QHBoxLayout()
        self.control_layout.addWidget(self.status_label)
        self.control_layout.addStretch()
        self.control_layout.addWidget(self.action_button)

        left_layout = QVBoxLayout()
        left_layout.addLayout(self.control_layout)
        left_layout.addWidget(self.calibrators_view)
        # left_layout.SetFixedWidth(600)

        right_layout = QVBoxLayout()
        right_layout.addWidget(self.initial_tfs_group)
        right_layout.addWidget(self.calibration_tfs_group)
        right_layout.addWidget(self.final_tfs_group)

        self.main_layout.addLayout(left_layout)
        self.main_layout.addLayout(right_layout)

        self.tfs_graph_signal.connect(self.tf_graph_callback2)

        selector_view = CalibrationSelectorView()
        selector_view.selected_calibrator_signal.connect(self.on_selected_calibrator)

        self.hide()

    def on_selected_calibrator(self, project_name, calibrator_name):
        print(
            f"on_selected_calibrator: project_name={project_name} calibrator_name={calibrator_name}",
            flush=True,
        )
        self.launcher_configuration_view = LauncherConfigurationView(project_name, calibrator_name)
        self.launcher_configuration_view.launcher_parameters.connect(
            partial(self.launch_calibrators, project_name, calibrator_name)
        )
        pass

    def launch_calibrators(
        self, project_name: str, calibrator_name: str, launch_argument_dict: Dict
    ):
        # Show the main UI
        self.show()

        # Execute the launcher
        print(launch_argument_dict, flush=True)
        argument_list = [f"{k}:={v}" for k, v in launch_argument_dict.items()]

        package_share_directory = get_package_share_directory("new_extrinsic_calibration_manager")
        launcher_path = (
            package_share_directory
            + "/launch/"
            + project_name
            + "/"
            + calibrator_name
            + ".launch.xml"
        )

        command_list = ["ros2", "launch", launcher_path] + argument_list
        print(f"Launching calibrator with the following command: {command_list}", flush=True)
        self.process = subprocess.Popen(command_list)

        # Recover all the launcher arguments (in addition to user defined in launch_arguments)
        try:
            with open(launcher_path) as f:
                root_entity, parser = Parser.load(f)
        except Exception as e:
            print("Failed reading xml file. Either not-existent or invalid")
            raise e

        ld: LaunchDescription = parser.parse_description(root_entity)
        context = LaunchContext()
        context.launch_configurations.update(launch_argument_dict)

        for e in ld.entities:
            if isinstance(e, (DeclareLaunchArgument, SetLaunchConfiguration)):
                e.visit(context)

        print(context.launch_configurations)

        # Start the ROS interface
        self.ros_interface = RosInterface()
        self.ros_interface.set_tfs_graph_callback(self.tfs_graph_callback)

        # Create the calibrator wrapper
        self.calibrator = CalibratorRegistry.create_calibrator(
            project_name,
            calibrator_name,
            ros_interface=self.ros_interface,
            **context.launch_configurations,
        )
        self.calibrator.state_changed_signal.connect(self.on_calibrator_state_changed)
        self.calibrator.calibration_finished_signal.connect(self.on_calibration_finished)

        self.calibrators_model = CalibratorManagerModel(self.calibrator.get_service_wrappers())
        self.calibrators_view.setModel(self.calibrators_model)
        self.calibrators_view.resizeColumnsToContents()
        self.calibrators_view.setFixedWidth(800)

        self.ros_interface.spin()
        pass

    def on_calibrator_state_changed(self, state: CalibratorState):
        text_dict = {
            CalibratorState.WAITING_TFS: "Waiting for the required TFs to be available",
            CalibratorState.WAITING_SERVICES: "Waiting for calibration services",
            CalibratorState.READY: "Ready to calibrate",
            CalibratorState.CALIBRATING: "Calibrating...",
            CalibratorState.FINISHED: "Calibration finished",
        }

        self.status_label.setText(text_dict[state])

        if state == CalibratorState.READY:
            self.action_button.clicked.connect(self.on_calibration_request)
            self.action_button.setEnabled(True)
        elif state == CalibratorState.FINISHED:
            self.action_button.clicked.disconnect()
            self.action_button.clicked.connect(self.on_save_request)
            self.action_button.setEnabled(True)
            self.action_button.setText("Save calibration")
        else:
            self.action_button.setEnabled(False)

    def on_calibration_request(self):
        print("on_calibration_request", flush=True)
        self.calibrator.start_calibration()

    def on_calibration_finished(self):
        tf_dict = self.calibrator.get_calibration_results()
        processed_tf_dict = self.calibrator.get_processed_calibration_results()
        self.calibration_tfs_view.setTfs(tf_dict)

        # There are two main cases:
        # 1) The processed_tf_dict modify the original tfs
        # 2) The processed tf_dict makes a new structure, hence it must be displayed as such
        final_tfs_dict = copy.deepcopy(self.tfs_dict)

        changed_frames_dict = self.calibration_results = defaultdict(set)

        for parent, child_and_tf in processed_tf_dict.items():
            if parent not in final_tfs_dict:
                continue
            for (
                child,
                transform,
            ) in child_and_tf.items():
                if child in final_tfs_dict[parent]:
                    final_tfs_dict[parent][child] = transform
                    changed_frames_dict[parent].add(child)

        if len(changed_frames_dict) == len(processed_tf_dict):
            self.final_tfs_view.setTfs(
                final_tfs_dict,
                changed_frames_dict=changed_frames_dict,
                required_frames=self.calibrator.required_frames,
            )
        else:
            self.final_tfs_view.setTfs(
                processed_tf_dict,
                changed_frames_dict=None,
                required_frames=self.calibrator.required_frames,
            )

    def on_save_request(self):
        print("on_save_request", flush=True)

        output_path = QFileDialog.getSaveFileName(
            None, "Save File", f"{os.getcwd()}/calibration_results.yaml", "YAML files (*.yaml)"
        )

        print(output_path, flush=True)
        output_path = output_path[0]

        if output_path is None or output_path == "":
            print("Invalid path", flush=True)
            return

        print(f"Saving calibration results to {output_path}")
        self.calibrator.save_results(output_path, use_rpy=True)

    def tfs_graph_callback(self, tfs_dict):
        with self.lock:
            tfs_dict = copy.deepcopy(tfs_dict)

        self.tfs_graph_signal.emit(copy.deepcopy(tfs_dict))

    def tf_graph_callback2(self, tfs_dict):
        if self.calibrator.state != CalibratorState.WAITING_TFS:
            return

        for parent, children_and_tf_dict in tfs_dict.items():
            self.tfs_dict[parent].update(children_and_tf_dict)

        if self.tfs_dict and len(self.tfs_dict) > 0 and self.calibrator:
            self.initial_tfs_view.setTfs(
                self.tfs_dict, required_frames=self.calibrator.required_frames
            )
        # self.calibration_tfs_view.setTfs(self.tfs_dict)
        # self.final_tfs_view.setTfs(self.tfs_dict)

        # self.tf_view.plot.setTfs(self.tfs_msg.transforms)
        # print("SECOND", flush=True)


def main(args=None):
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""
    app = QApplication(sys.argv)

    rclpy.init(args=args)

    try:
        signal.signal(signal.SIGINT, sigint_handler)
        calibration_manager = NewExtrinsicCalibrationManager()  # noqa: F841

        sys.exit(app.exec_())
    except (KeyboardInterrupt, SystemExit):
        print("Received sigint. Quitting...")
        rclpy.shutdown()


def sigint_handler(*args):
    QApplication.quit()


if __name__ == "__main__":
    main()
