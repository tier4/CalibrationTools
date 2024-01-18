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

import json
import logging
import os
import signal
import sys

from PySide2.QtCore import Qt
from PySide2.QtCore import Signal
from PySide2.QtGui import QPixmap
from PySide2.QtWidgets import QApplication
from PySide2.QtWidgets import QCheckBox
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QDoubleSpinBox
from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QFrame
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QSpinBox
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from interactive_camera_lidar_calibrator.calibrator import Calibrator
from interactive_camera_lidar_calibrator.ros_interface import InteractiveCalibratorRosInterface
from interactive_camera_lidar_calibrator.utils import camera_lidar_calibrate_intrinsics
import numpy as np
import rclpy
from rosidl_runtime_py.convert import message_to_ordereddict
from tier4_calibration_views.image_view_ui import ImageViewUI


class InteractiveCalibratorUI(ImageViewUI):
    object_point_signal = Signal(float, float, float)

    def __init__(self, ros_interface: InteractiveCalibratorRosInterface):
        # Calibrator
        self.calibrator = Calibrator()

        # Calibration variables
        self.object_calibration_points = []
        self.image_calibration_points = []
        self.external_object_calibration_points = []
        self.external_image_calibration_points = []
        self.current_object_point = None
        self.current_image_point = None
        self.optimized_camera_info = None
        self.calibration_possible = False
        self.calibrated_error = np.inf
        self.calibration_api_request_received = False

        super().__init__(ros_interface)

        self.setWindowTitle("Interactive camera-lidar calibration tool")

        # ROS Interface
        ros_interface.set_object_point_callback(self.object_point_ros_callback)

        ros_interface.set_calibration_api_request_received_callback(
            self.calibration_api_request_received_callback
        )
        ros_interface.set_calibration_api_request_sent_callback(
            self.calibration_api_request_sent_callback
        )

        self.object_point_signal.connect(self.object_point_callback)

        self.show()

    def make_left_menu(self):
        # Calibration tools API group
        self.make_calibration_tools_api()

        # Calibration options group
        self.make_calibration_options()

        # Data collection group
        self.make_data_collection_options()

        self.left_menu_widget = QWidget(self.central_widget)
        self.left_menu_widget.setFixedWidth(200)
        self.left_menu_layout = QVBoxLayout(self.left_menu_widget)

        # self.menu_layout.addWidget(label)
        self.left_menu_layout.addWidget(self.calibration_api_group)
        self.left_menu_layout.addWidget(self.data_collection_options_group)
        self.left_menu_layout.addWidget(self.calibration_status_group)
        self.left_menu_layout.addWidget(self.calibration_options_group)

    def make_calibration_tools_api(self):
        self.calibration_api_group = QGroupBox("Calibration API")

        self.calibration_api_label = QLabel()
        self.calibration_api_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.calibration_api_label.setStyleSheet(
            "QLabel { background-color : red; color : black; }"
        )
        self.calibration_api_label.setText("Disconnected")
        self.calibration_api_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        def calibration_api_button_callback():
            self.ros_interface.send_calibration_api_result(self.calibrated_error)

        self.calibration_api_button = QPushButton("Send calibration")
        self.calibration_api_button.setEnabled(False)
        self.calibration_api_button.clicked.connect(calibration_api_button_callback)

        calibration_api_layout = QVBoxLayout()
        calibration_api_layout.addWidget(self.calibration_api_label)
        calibration_api_layout.addWidget(self.calibration_api_button)
        # calibration_api_layout.addStretch(1)
        self.calibration_api_group.setLayout(calibration_api_layout)

        # Calibration status group
        self.calibration_status_group = QGroupBox("Calibration status")
        self.calibration_status_group.setFlat(True)

        calibration_status_helper_label = QLabel()
        calibration_status_helper_label.setText("(calibration vs. tf source)")

        self.calibration_status_points_label = QLabel()
        self.calibration_status_points_label.setText("#points: ")
        self.calibration_status_points_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.calibration_status_error_label = QLabel()
        self.calibration_status_error_label.setText("r.error: ")
        self.calibration_status_error_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.calibration_status_inliers_label = QLabel()
        self.calibration_status_inliers_label.setText("inliers: ")
        self.calibration_status_inliers_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.state_1_message = (
            "To add a calibration pair\nfirst click the 3d point."
            + "\nTo delete a calibration\npoint, click it in the\nimage"
        )

        user_message_label = QLabel()
        user_message_label.setText(self.state_1_message)
        user_message_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        save_calibration_button = QPushButton("Save calibration")
        save_calibration_button.clicked.connect(self.save_calibration_callback)

        load_calibration_button = QPushButton("Load calibration")
        load_calibration_button.clicked.connect(self.load_calibration_callback)

        calibration_status_layout = QVBoxLayout()
        calibration_status_layout.addWidget(calibration_status_helper_label)
        calibration_status_layout.addWidget(self.calibration_status_points_label)
        calibration_status_layout.addWidget(self.calibration_status_error_label)
        calibration_status_layout.addWidget(self.calibration_status_inliers_label)
        calibration_status_layout.addWidget(user_message_label)
        calibration_status_layout.addWidget(save_calibration_button)
        calibration_status_layout.addWidget(load_calibration_button)
        # calibration_status_layout.addStretch(1)
        self.calibration_status_group.setLayout(calibration_status_layout)

    def make_calibration_options(self):
        self.calibration_options_group = QGroupBox("Calibration options")
        self.calibration_options_group.setFlat(True)

        self.calibration_button = QPushButton("Calibrate extrinsics")
        self.calibration_button.clicked.connect(self.calibration_callback)
        self.calibration_button.setEnabled(False)

        def calibration_intrinsics_callback():
            self.optimized_camera_info = camera_lidar_calibrate_intrinsics(
                np.array(self.object_calibration_points + self.external_object_calibration_points),
                np.array(self.image_calibration_points + self.external_image_calibration_points),
            )
            self.use_optimized_intrinsics_checkbox.setEnabled(True)

        self.calibration2_button = QPushButton("Calibrate intrinsics\n(experimental)")
        self.calibration2_button.clicked.connect(calibration_intrinsics_callback)
        self.calibration2_button.setEnabled(False)

        pnp_method_label = QLabel("Pnp solver:")
        self.pnp_method_combobox = QComboBox()
        self.pnp_method_combobox.addItem("Iterative")
        self.pnp_method_combobox.addItem("SQPNP")

        def use_ransac_callback(value):
            if self.camera_info is None:
                return

            if value == Qt.Checked:
                self.render_inliers_checkbox.setChecked(False)
                self.render_inliers_checkbox.setEnabled(True)
            else:
                self.render_inliers_checkbox.setChecked(False)
                self.render_inliers_checkbox.setEnabled(False)

        self.use_ransac_checkbox = QCheckBox("Use RANSAC")
        self.use_ransac_checkbox.stateChanged.connect(use_ransac_callback)
        self.use_ransac_checkbox.setChecked(False)

        def use_optimized_intrinsics_callback(state):
            if state == Qt.Checked:
                self.image_view.set_camera_info(
                    self.optimized_camera_info.k, self.optimized_camera_info.d
                )
                self.calibrator.set_camera_info(
                    self.optimized_camera_info.k, self.optimized_camera_info.d
                )
            else:
                self.image_view.set_camera_info(self.camera_info.k, self.camera_info.d)
                self.calibrator.set_camera_info(self.camera_info.k, self.camera_info.d)

        self.use_optimized_intrinsics_checkbox = QCheckBox("Use optimized Intrinsics")
        self.use_optimized_intrinsics_checkbox.stateChanged.connect(
            use_optimized_intrinsics_callback
        )
        self.use_optimized_intrinsics_checkbox.setEnabled(False)
        self.use_optimized_intrinsics_checkbox.setChecked(False)

        def pnp_min_points_callback():
            self.calibration_possible = (
                len(self.image_calibration_points) + len(self.external_image_calibration_points)
                >= self.pnp_min_points_spinbox.value()
            )

            self.calibration_button.setEnabled(self.calibration_possible)
            self.calibration2_button.setEnabled(self.calibration_possible)

        pnp_min_points_label = QLabel("Minimum pnp\n points")
        self.pnp_min_points_spinbox = QSpinBox()
        self.pnp_min_points_spinbox.valueChanged.connect(pnp_min_points_callback)
        self.pnp_min_points_spinbox.setRange(4, 100)
        self.pnp_min_points_spinbox.setSingleStep(1)
        self.pnp_min_points_spinbox.setValue(6)

        def ransac_inlier_error_callback(value):
            self.image_view.set_inlier_distance(value)

        ransac_inlier_error_label = QLabel("RANSAC inlier\nerror (px)")
        self.ransac_inlier_error_spinbox = QDoubleSpinBox()
        self.ransac_inlier_error_spinbox.valueChanged.connect(ransac_inlier_error_callback)
        self.ransac_inlier_error_spinbox.setRange(0.0, 1000.0)
        self.ransac_inlier_error_spinbox.setSingleStep(0.1)
        self.ransac_inlier_error_spinbox.setValue(10.0)

        calibration_options_layout = QVBoxLayout()
        calibration_options_layout.addWidget(self.calibration_button)
        calibration_options_layout.addWidget(self.calibration2_button)
        calibration_options_layout.addWidget(pnp_method_label)
        calibration_options_layout.addWidget(self.pnp_method_combobox)
        calibration_options_layout.addWidget(self.use_ransac_checkbox)
        calibration_options_layout.addWidget(self.use_optimized_intrinsics_checkbox)
        calibration_options_layout.addWidget(pnp_min_points_label)
        calibration_options_layout.addWidget(self.pnp_min_points_spinbox)
        calibration_options_layout.addWidget(ransac_inlier_error_label)
        calibration_options_layout.addWidget(self.ransac_inlier_error_spinbox)
        calibration_options_layout.addStretch(1)
        self.calibration_options_group.setLayout(calibration_options_layout)

    def make_data_collection_options(self):
        self.data_collection_options_group = QGroupBox("Data collection\noptions")
        self.data_collection_options_group.setFlat(True)

        def pause_start_callback():
            if self.ros_interface.is_paused():
                self.pause_start_button.setText("Pause")
                self.ros_interface.set_paused(False)
            else:
                self.pause_start_button.setText("Play")
                self.ros_interface.set_paused(True)

        self.pause_start_button = QPushButton("Pause")
        self.pause_start_button.setEnabled(True)
        self.pause_start_button.clicked.connect(pause_start_callback)

        def screenshot_callback():
            pixmap_roi = QPixmap(self.graphics_view.size())
            self.graphics_view.render(pixmap_roi)

            pixmap_raw = self.image_view.take_screenshot()

            # check if exists then save
            index = 0

            while True:
                file_name_raw = f"screenshot_{index:04d}_raw.png"
                file_name_roi = f"screenshot_{index:04d}_roi.png"

                if not os.path.exists(file_name_raw):
                    pixmap_raw.save(file_name_raw, "PNG", -1)
                    pixmap_roi.save(file_name_roi, "PNG", -1)
                    break

                index += 1

        self.screenshot_button = QPushButton("Take Screenshot")
        self.screenshot_button.setEnabled(True)
        self.screenshot_button.clicked.connect(screenshot_callback)

        self.publish_tf_checkbox = QCheckBox("Publish tf")
        self.publish_tf_checkbox.stateChanged.connect(self.ros_interface.set_publish_tf)
        self.publish_tf_checkbox.setChecked(True)

        data_collection_options_layout = QVBoxLayout()
        data_collection_options_layout.addWidget(self.pause_start_button)
        data_collection_options_layout.addWidget(self.screenshot_button)
        data_collection_options_layout.addWidget(self.publish_tf_checkbox)
        data_collection_options_layout.addStretch(1)
        self.data_collection_options_group.setLayout(data_collection_options_layout)

    def sensor_data_callback(self):
        super().sensor_data_callback()
        with self.lock:
            self.calibrator.set_camera_info(self.camera_info_tmp.k, self.camera_info_tmp.d)

    def tf_source_callback(self, string):
        super().tf_source_callback(string)
        self.update_calibration_status()

    def save_calibration_callback(self):
        output_folder = QFileDialog.getExistingDirectory(
            None,
            "Select directory to save the calibration result",
            ".",
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )

        if output_folder is None or output_folder == "":
            return

        logging.info(output_folder)

        object_points = np.array(
            self.object_calibration_points + self.external_object_calibration_points,
            dtype=np.float64,
        )
        image_points = np.array(
            self.image_calibration_points + self.external_image_calibration_points, dtype=np.float64
        )

        num_points, dim = object_points.shape
        assert dim == 3
        assert num_points == image_points.shape[0]
        assert 2 == image_points.shape[1]

        np.savetxt(
            os.path.join(output_folder, "object_points.txt"), object_points
        )  # cSpell:ignore savetxt
        np.savetxt(os.path.join(output_folder, "image_points.txt"), image_points)

        if self.optimized_camera_info is not None:
            d = message_to_ordereddict(self.optimized_camera_info)

            with open(os.path.join(output_folder, "optimized_camera_info.json"), "w") as f:
                f.write(json.dumps(d, indent=4, sort_keys=False))

        self.ros_interface.save_calibration_tfs(output_folder)
        pass

    def load_calibration_callback(self):
        input_dir = QFileDialog.getExistingDirectory(
            None,
            "Select directory to load calibration points from",
            ".",
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )

        if input_dir is None or input_dir == "":
            return

        logging.info(input_dir)

        object_calibration_points = np.loadtxt(
            os.path.join(input_dir, "object_points.txt")
        )  # cSpell:ignore loadtxt
        image_calibration_points = np.loadtxt(os.path.join(input_dir, "image_points.txt"))

        self.object_calibration_points = list(object_calibration_points)
        self.image_calibration_points = list(image_calibration_points)

        logging.info(self.object_calibration_points)
        logging.info(self.image_calibration_points)

        assert len(self.image_calibration_points) == len(self.object_calibration_points)

        self.calibration_possible = (
            len(self.image_calibration_points) + len(self.external_image_calibration_points)
            >= self.pnp_min_points_spinbox.value()
        )

        self.calibration_button.setEnabled(self.calibration_possible)
        self.calibration2_button.setEnabled(self.calibration_possible)

        self.calibration_callback()

        self.image_view.set_calibration_points(
            self.object_calibration_points, self.image_calibration_points
        )
        pass

    def calibration_callback(self):
        if self.camera_info is None:
            return

        # Configure the calibrator
        self.calibrator.set_inlier_error(self.ransac_inlier_error_spinbox.value())
        self.calibrator.set_method(self.pnp_method_combobox.currentText().lower())
        self.calibrator.set_min_points(self.pnp_min_points_spinbox.value())
        self.calibrator.set_ransac(self.use_ransac_checkbox.isChecked())

        # Calibrate
        object_calibration_points = (
            self.object_calibration_points + self.external_object_calibration_points
        )
        image_calibration_points = (
            self.image_calibration_points + self.external_image_calibration_points
        )

        if len(object_calibration_points) == 0 or len(image_calibration_points) == 0:
            return

        transform = self.calibrator.calibrate(object_calibration_points, image_calibration_points)

        if transform is None:
            return

        if self.calibrated_transform is None:
            self.tf_source_combobox.addItem("Calibrator")

        self.calibrated_transform = transform

        self.tf_source_callback(self.tf_source_combobox.currentText())

        self.calibration_api_button.setEnabled(
            self.calibration_api_request_received and self.calibrated_transform is not None
        )
        self.ros_interface.set_camera_lidar_transform(transform)

    def update_calibration_status(self):
        object_calibration_points = (
            self.object_calibration_points + self.external_object_calibration_points
        )
        image_calibration_points = (
            self.image_calibration_points + self.external_image_calibration_points
        )

        calibration_points_available = len(object_calibration_points) > 0

        if self.calibrated_transform is not None and calibration_points_available:
            (
                calibrated_error,
                calibrated_inliers,
            ) = self.calibrator.calculate_reproj_error(  # cSpell:ignore reproj
                object_calibration_points,
                image_calibration_points,
                transform_matrix=self.calibrated_transform,
            )
            calibrated_error_string = f"{calibrated_error:.2f}"
            self.calibrated_error = calibrated_error
        else:
            calibrated_error_string = ""
            calibrated_inliers = np.array([])

        if self.source_transform is not None and calibration_points_available:
            source_error, source_inliers = self.calibrator.calculate_reproj_error(
                object_calibration_points,
                image_calibration_points,
                transform_matrix=self.source_transform,
            )
            source_error_string = f"{source_error:.2f}"
        else:
            source_error_string = ""
            source_inliers = np.array([])

        self.calibration_status_points_label.setText(f"#points: {len(object_calibration_points)}")
        self.calibration_status_error_label.setText(
            f"R.error: {calibrated_error_string} / {source_error_string}"
        )
        self.calibration_status_inliers_label.setText(
            f"inliers:  {int(calibrated_inliers.sum())} / {int(source_inliers.sum())}"
        )

    def object_point_ros_callback(self, point):
        assert np.prod(point.shape) == 3
        point = point.reshape(
            3,
        )
        self.object_point_signal.emit(point[0], point[1], point[2])

    def calibration_api_request_received_callback(self):
        self.calibration_api_label.setStyleSheet(
            "QLabel { background-color : yellow; color : black; }"
        )
        self.calibration_api_label.setText("Waiting calibration")

        self.calibration_api_request_received = True

        self.calibration_api_button.setEnabled(
            self.calibration_api_request_received and self.calibrated_transform is not None
        )

    def calibration_api_request_sent_callback(self):
        self.calibration_api_label.setStyleSheet(
            "QLabel { background-color : green; color : black; }"
        )
        self.calibration_api_label.setText("Calibration sent")

        self.calibration_api_request_received = False
        self.calibration_api_button.setEnabled(False)

    def image_click_callback(self, x, y):
        p = np.array([x, y]).reshape((2,))

        if self.current_object_point is None:
            self.delete_calibration_points(p)
            return

        self.image_calibration_points.append(p)
        self.object_calibration_points.append(
            self.current_object_point.reshape(
                3,
            )
        )

        self.image_view.set_calibration_points(
            self.object_calibration_points, self.image_calibration_points
        )

        self.calibration_callback()

        self.current_object_point = None
        self.image_view.set_current_point(None)

        self.calibration_possible = (
            len(self.image_calibration_points) + len(self.external_image_calibration_points)
            >= self.pnp_min_points_spinbox.value()
        )

        self.calibration_button.setEnabled(self.calibration_possible)
        self.calibration2_button.setEnabled(self.calibration_possible)

        self.image_view.update()

    def delete_calibration_points(self, p):
        if len(self.object_calibration_points) == 0:
            return

        p = p.reshape(1, 2)

        # Detection deletion feature
        tolerance = 15  # px

        object_points = np.array(self.object_calibration_points)
        image_points = np.array(self.image_calibration_points)

        indexes = np.linalg.norm(image_points - p, axis=1) > tolerance

        object_points = object_points[indexes, :]
        image_points = image_points[indexes, :]

        self.object_calibration_points = list(object_points)
        self.image_calibration_points = list(image_points)

        self.calibration_callback()
        self.image_view.set_calibration_points(
            self.object_calibration_points, self.image_calibration_points
        )

    def object_point_callback(self, x, y, z):
        point_array = np.array([x, y, z], np.float64).reshape(1, 3)

        self.image_view.set_current_point(point_array)
        self.image_view.update()

        self.current_object_point = point_array

    def external_calibration_points_callback(self):
        super().external_calibration_points_callback()

        self.calibration_possible = (
            len(self.image_calibration_points) + len(self.external_image_calibration_points)
            >= self.pnp_min_points_spinbox.value()
        )

        self.calibration_button.setEnabled(self.calibration_possible)
        self.calibration2_button.setEnabled(self.calibration_possible)


def main(args=None):
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""
    app = QApplication(sys.argv)

    rclpy.init(args=args)

    try:
        signal.signal(signal.SIGINT, sigint_handler)

        ros_interface = InteractiveCalibratorRosInterface()
        ex = InteractiveCalibratorUI(ros_interface)  # noqa: F841

        ros_interface.spin()

        sys.exit(app.exec_())
    except (KeyboardInterrupt, SystemExit):
        logging.info("Received sigint. Quitting...")
        rclpy.shutdown()


def sigint_handler(*args):
    QApplication.quit()


if __name__ == "__main__":
    main()
