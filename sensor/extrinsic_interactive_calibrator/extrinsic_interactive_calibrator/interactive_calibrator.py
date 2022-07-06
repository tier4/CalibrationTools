#!/usr/bin/env python3

# Copyright 2020 Tier IV, Inc.
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

import copy, json, os, signal, sys, rclpy, threading
import numpy as np

from extrinsic_interactive_calibrator.calibrator import Calibrator
from extrinsic_interactive_calibrator.image_view import CustomQGraphicsView, ImageView
from extrinsic_interactive_calibrator.ros_interface import RosInterface
from rosidl_runtime_py.convert import message_to_ordereddict
from PySide2.QtCore import Qt, Signal
from PySide2.QtGui import QImage, QPixmap
from PySide2.QtWidgets import QApplication, QCheckBox, QComboBox, QDoubleSpinBox, QFileDialog, \
    QFrame, QGraphicsScene, QGraphicsView, QGroupBox, QHBoxLayout, QLabel, QMainWindow, \
    QPushButton, QSpinBox, QVBoxLayout, QWidget
class InteractiveCalibratorUI(QMainWindow):

    sensor_data_signal = Signal()
    transform_signal = Signal()
    object_point_signal = Signal(float, float, float)
    external_calibration_points_signal = Signal()
    optimized_intrinsics_signal = Signal()

    def __init__(self, ros_interface):
        super().__init__()
        self.setWindowTitle("Interactive camera-lidar calibration tool")

        # ROS Interface
        self.ros_interface = ros_interface
        self.ros_interface.set_sensor_data_callback(self.sensor_data_ros_callback)
        self.ros_interface.set_transform_callback(self.transform_ros_callback)
        self.ros_interface.set_object_point_callback(self.object_point_ros_callback)
        self.ros_interface.set_external_calibration_points_callback(
            self.external_calibration_points_ros_callback)
        self.ros_interface.set_optimize_camera_intrinsics_status_callback(
            self.optimize_camera_intrinsics_status_callback)
        self.ros_interface.set_optimize_camera_intrinsics_result_callback(
            self.optimize_camera_intrinsics_result_callback)
        self.ros_interface.set_calibration_api_request_received_callback(
            self.calibration_api_request_received_callback)
        self.ros_interface.set_calibration_api_request_sent_callback(
            self.calibration_api_request_sent_callback)

        self.sensor_data_signal.connect(self.sensor_data_callback)
        self.transform_signal.connect(self.transform_callback)
        self.object_point_signal.connect(self.object_point_callback)
        self.external_calibration_points_signal.connect(self.external_calibration_points_callback)
        self.optimized_intrinsics_signal.connect(self.optimized_camera_info_callback)

        # Threading variables
        self.lock = threading.RLock()
        self.transform_tmp = None
        self.external_object_calibration_points_tmp = None
        self.external_image_calibration_points_tmp = None
        self.pixmap_tmp = None
        self.camera_info_tmp = None
        self.pointcloud_tmp = None
        self.optimized_camera_info_tmp = None

        # Calibrator
        self.calibrator = Calibrator()

        # Calibration variables
        self.object_calibration_points = []
        self.image_calibration_points = []
        self.external_object_calibration_points = []
        self.external_image_calibration_points = []
        self.current_object_point = None
        self.current_image_point = None
        self.camera_info = None
        self.optimized_camera_info = None
        self.initial_transform = None
        self.current_transform = None
        self.calibrated_transform = None
        self.source_transform = None
        self.optimize_camera_intrinsics_status = False
        self.optimize_camera_intrinsics_waiting = False
        self.calibration_possible = False
        self.calibrated_error = np.inf
        self.calibration_api_request_received = False

        # Parent widget
        self.central_widget = QWidget(self)
        #self.central_widget.resize(1000,1000)

        self.setCentralWidget(self.central_widget)
        self.layout = QHBoxLayout(self.central_widget)

        # Image View
        self.make_image_view()

        # Menu Widgets
        self.left_menu_widget = QWidget(self.central_widget)
        self.left_menu_widget.setFixedWidth(200)
        self.left_menu_layout = QVBoxLayout(self.left_menu_widget)

        self.right_menu_widget = QWidget(self.central_widget)
        self.right_menu_widget.setFixedWidth(210)
        self.right_menu_layout = QVBoxLayout(self.right_menu_widget)

        # Calibration tools API group
        self.make_calibration_tools_api()

        # Calibration options group
        self.make_calibration_options()

        # Data collection group
        self.make_data_collection_options()

        # Visualization group
        self.make_vizualization_options()

        #self.menu_layout.addWidget(label)
        self.left_menu_layout.addWidget(self.calibration_api_group)
        self.left_menu_layout.addWidget(self.data_collection_options_group)
        self.left_menu_layout.addWidget(self.calibration_status_group)
        self.left_menu_layout.addWidget(self.calibration_options_group)

        self.right_menu_layout.addWidget(self.visualization_options_group)

        self.layout.addWidget(self.graphics_view)
        #self.layout.addWidget(self.image_view)

        self.layout.addWidget(self.left_menu_widget)
        self.layout.addWidget(self.right_menu_widget)

        self.show()

    def make_image_view(self):

        self.image_view = ImageView()
        #self.image_view.set_pixmap(pixmap)
        self.image_view.clicked_signal.connect(self.image_click_callback)

        # We need the view to control the zoom
        self.graphics_view = CustomQGraphicsView(self.central_widget)
        self.graphics_view.setCacheMode(QGraphicsView.CacheBackground)
        self.graphics_view.setViewportUpdateMode(QGraphicsView.BoundingRectViewportUpdate)

        # The scene contains the items
        self.scene = QGraphicsScene()

        # Add the item into the scene
        self.scene.addItem(self.image_view)

        # Add the scene into the view
        self.graphics_view.setScene(self.scene)


    def make_calibration_tools_api(self):
        self.calibration_api_group = QGroupBox("Calibration API")

        self.calibration_api_label = QLabel()
        self.calibration_api_label.setFrameStyle(QFrame.Panel | QFrame.Sunken)
        self.calibration_api_label.setStyleSheet(
            "QLabel { background-color : red; color : black; }")
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
        #calibration_api_layout.addStretch(1)
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
        self.calibration_status_error_label.setText("Reproj error: ")
        self.calibration_status_error_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.calibration_status_inliers_label = QLabel()
        self.calibration_status_inliers_label.setText("inliers: ")
        self.calibration_status_inliers_label.setAlignment(Qt.AlignTop | Qt.AlignLeft)

        self.state_1_message = f"To add a calibration pair\nfirst click the 3d point." + \
            "\nTo delete a calibration\npoint, click it in the\nimage"

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
        #calibration_status_layout.addStretch(1)
        self.calibration_status_group.setLayout(calibration_status_layout)

    def make_calibration_options(self):
        self.calibration_options_group = QGroupBox("Calibration options")
        self.calibration_options_group.setFlat(True)

        self.calibration_button = QPushButton("Calibrate extrinsics")
        self.calibration_button.clicked.connect(self.calibration_callback)
        self.calibration_button.setEnabled(False)

        def calibration_intrinsics_callback():
            self.ros_interface.optimize_camera_intrinsics(
                self.object_calibration_points + self.external_object_calibration_points,
                self.image_calibration_points + self.external_image_calibration_points)

            self.calibration2_button.setEnabled(False)
            self.calibration2_button.setText("Optimizing...")
            self.optimize_camera_intrinsics_waiting = True
            assert self.optimize_camera_intrinsics_status == True

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
                    self.optimized_camera_info.k, self.optimized_camera_info.d)
                self.calibrator.set_camera_info(
                    self.optimized_camera_info.k, self.optimized_camera_info.d)
            else:
                self.image_view.set_camera_info(self.camera_info.k, self.camera_info.d)
                self.calibrator.set_camera_info(self.camera_info.k, self.camera_info.d)

        self.use_optimized_intrinsics_checkbox = QCheckBox("Use optimized Intrinsics")
        self.use_optimized_intrinsics_checkbox.stateChanged.connect(use_optimized_intrinsics_callback)
        self.use_optimized_intrinsics_checkbox.setEnabled(False)
        self.use_optimized_intrinsics_checkbox.setChecked(False)

        def pnp_min_points_callback():
            self.calibration_possible = \
                len(self.image_calibration_points) + len(self.external_image_calibration_points) \
                    >= self.pnp_min_points_spinbox.value()

            self.calibration_button.setEnabled(self.calibration_possible)
            self.calibration2_button.setEnabled(
                self.calibration_possible and
                self.optimize_camera_intrinsics_status and
                not self.optimize_camera_intrinsics_waiting)

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

        def republish_data_callback(state):
            self.ros_interface.set_republish_data(state == Qt.Checked)

        self.republish_data_checkbox = QCheckBox("Republish calibration\ndata")
        self.republish_data_checkbox.stateChanged.connect(republish_data_callback)
        self.republish_data_checkbox.setChecked(True)

        def republish_data_callback(state):
            self.ros_interface.set_publish_tf(state == Qt.Checked)

        self.publish_tf_checkbox = QCheckBox("Publish tf")
        self.publish_tf_checkbox.stateChanged.connect(republish_data_callback)
        self.publish_tf_checkbox.setChecked(True)



        data_collection_options_layout = QVBoxLayout()
        data_collection_options_layout.addWidget(self.pause_start_button)
        data_collection_options_layout.addWidget(self.screenshot_button)
        data_collection_options_layout.addWidget(self.republish_data_checkbox)
        data_collection_options_layout.addWidget(self.publish_tf_checkbox)
        data_collection_options_layout.addStretch(1)
        self.data_collection_options_group.setLayout(data_collection_options_layout)

    def make_vizualization_options(self):
        self.visualization_options_group = QGroupBox("Visualization options")
        self.visualization_options_group.setFlat(True)

        tf_source_label = QLabel("TF source:")
        self.tf_source_combobox = QComboBox()
        self.tf_source_combobox.currentTextChanged.connect(self.tf_source_callback)

        def marker_type_callback(value):
            self.image_view.set_marker_type(value)

        marker_type_label = QLabel("Marker type:")
        marker_type_combobox = QComboBox()
        marker_type_combobox.currentTextChanged.connect(marker_type_callback)
        marker_type_combobox.addItem("Circles")
        marker_type_combobox.addItem("Rectangles")

        def marker_units_callback(value):
            self.image_view.set_marker_units(value)

        marker_units_label = QLabel("Marker units:")
        marker_units_combobox = QComboBox()
        marker_units_combobox.currentTextChanged.connect(marker_units_callback)
        marker_units_combobox.addItem("Meters")
        marker_units_combobox.addItem("Pixels")

        def marker_color_callback(value):
            self.image_view.set_color_channel(value)

        marker_color_label = QLabel("Marker color channel:")
        marker_color_combobox = QComboBox()
        marker_color_combobox.currentTextChanged.connect(marker_color_callback)
        marker_color_combobox.addItem("Intensity")
        marker_color_combobox.addItem("X")
        marker_color_combobox.addItem("Y")
        marker_color_combobox.addItem("Z")

        def marker_pixels_callback(value):
            self.image_view.set_marker_size_pixels(value)

        marker_pixels_label = QLabel("Marker size (px)")
        marker_pixels_spinbox = QSpinBox()
        marker_pixels_spinbox.valueChanged.connect(marker_pixels_callback)
        marker_pixels_spinbox.setRange(4, 100)
        marker_pixels_spinbox.setSingleStep(1)
        marker_pixels_spinbox.setValue(6)

        def marker_meters_callback(value):
            self.image_view.set_marker_size_meters(value)

        marker_meters_label = QLabel("Marker size (m)")
        marker_meters_spinbox = QDoubleSpinBox()
        marker_meters_spinbox.valueChanged.connect(marker_meters_callback)
        marker_meters_spinbox.setRange(0.01, 1.0)
        marker_meters_spinbox.setSingleStep(0.01)
        marker_meters_spinbox.setValue(0.05)

        def rainbow_distance_callback(value):
            self.image_view.set_rainbow_distance(value)

        rainbow_distance_label = QLabel("Rainbow distance (m)")
        rainbow_distance_spinbox = QDoubleSpinBox()
        rainbow_distance_spinbox.valueChanged.connect(rainbow_distance_callback)
        rainbow_distance_spinbox.setRange(0.0, 1000.0)
        rainbow_distance_spinbox.setSingleStep(0.1)
        rainbow_distance_spinbox.setValue(10.0)

        def rainbow_offset_callback(value):
            self.image_view.set_rainbow_offset(value)

        rainbow_offset_label = QLabel("Rainbow offset")
        rainbow_offset_spinbox = QDoubleSpinBox()
        rainbow_offset_spinbox.valueChanged.connect(rainbow_offset_callback)
        rainbow_offset_spinbox.setRange(0.0, 1.0)
        rainbow_offset_spinbox.setSingleStep(0.05)
        rainbow_offset_spinbox.setValue(0.0)

        def rendering_alpha_callback(value):
            self.image_view.set_rendering_alpha(value)

        rendering_alpha_label = QLabel("Rendering alpha")
        rendering_alpha_spinbox = QDoubleSpinBox()
        rendering_alpha_spinbox.valueChanged.connect(rendering_alpha_callback)
        rendering_alpha_spinbox.setRange(0.0, 1.0)
        rendering_alpha_spinbox.setSingleStep(0.05)
        rendering_alpha_spinbox.setValue(1.0)

        def marker_subsample_callback(value):
            self.image_view.set_subsample_factor(value)

        marker_subsample_label = QLabel("PC subsample factor")
        marker_subsample_spinbox = QSpinBox()
        marker_subsample_spinbox.valueChanged.connect(marker_subsample_callback)
        marker_subsample_spinbox.setRange(1, 10)
        marker_subsample_spinbox.setSingleStep(1)
        marker_subsample_spinbox.setValue(4)

        def rendering_min_distance_callback(value):
            self.image_view.set_min_rendering_distance(value)

        rendering_min_distance_label = QLabel("Min rendering distance (m)")
        rendering_min_distance_spinbox = QDoubleSpinBox()
        rendering_min_distance_spinbox.valueChanged.connect(rendering_min_distance_callback)
        rendering_min_distance_spinbox.setRange(0.01, 100.0)
        rendering_min_distance_spinbox.setSingleStep(0.1)
        rendering_min_distance_spinbox.setValue(0.1)

        def rendering_max_distance_callback(value):
            self.image_view.set_max_rendering_distance(value)

        rendering_max_distance_label = QLabel("Max rendering distance (m)")
        rendering_max_distance_spinbox = QDoubleSpinBox()
        rendering_max_distance_spinbox.valueChanged.connect(rendering_max_distance_callback)
        rendering_max_distance_spinbox.setRange(0.01, 100.0)
        rendering_max_distance_spinbox.setSingleStep(0.1)
        rendering_max_distance_spinbox.setValue(100.0)

        def render_pointcloud_callback(value):
            self.image_view.set_draw_pointcloud(value == Qt.Checked)

        render_pointcloud_checkbox = QCheckBox("Render pointcloud")
        render_pointcloud_checkbox.stateChanged.connect(render_pointcloud_callback)
        render_pointcloud_checkbox.setChecked(True)

        def render_calibration_points_callback(value):
            self.image_view.set_draw_calibration_points(value == Qt.Checked)

        render_calibration_points_checkbox = QCheckBox("Render calibration points")
        render_calibration_points_checkbox.stateChanged.connect(render_calibration_points_callback)
        render_calibration_points_checkbox.setChecked(True)

        def render_inliers_callback(value):
            self.image_view.set_draw_inliers(value == Qt.Checked)

        self.render_inliers_checkbox = QCheckBox("Render inliers")
        self.render_inliers_checkbox.stateChanged.connect(render_inliers_callback)
        self.render_inliers_checkbox.setChecked(False)
        self.render_inliers_checkbox.setEnabled(False)


        visualization_options_layout = QVBoxLayout()
        visualization_options_layout.addWidget(tf_source_label)
        visualization_options_layout.addWidget(self.tf_source_combobox)
        visualization_options_layout.addWidget(marker_type_label)
        visualization_options_layout.addWidget(marker_type_combobox)
        visualization_options_layout.addWidget(marker_units_label)
        visualization_options_layout.addWidget(marker_units_combobox)
        visualization_options_layout.addWidget(marker_color_label)
        visualization_options_layout.addWidget(marker_color_combobox)
        visualization_options_layout.addWidget(render_pointcloud_checkbox)
        visualization_options_layout.addWidget(render_calibration_points_checkbox)
        visualization_options_layout.addWidget(self.render_inliers_checkbox)

        visualization_options_layout.addWidget(marker_pixels_label)
        visualization_options_layout.addWidget(marker_pixels_spinbox)
        visualization_options_layout.addWidget(marker_meters_label)
        visualization_options_layout.addWidget(marker_meters_spinbox)
        visualization_options_layout.addWidget(rainbow_distance_label)
        visualization_options_layout.addWidget(rainbow_distance_spinbox)
        visualization_options_layout.addWidget(rainbow_offset_label)
        visualization_options_layout.addWidget(rainbow_offset_spinbox)
        visualization_options_layout.addWidget(rendering_alpha_label)
        visualization_options_layout.addWidget(rendering_alpha_spinbox)
        visualization_options_layout.addWidget(marker_subsample_label)
        visualization_options_layout.addWidget(marker_subsample_spinbox)
        visualization_options_layout.addWidget(rendering_min_distance_label)
        visualization_options_layout.addWidget(rendering_min_distance_spinbox)
        visualization_options_layout.addWidget(rendering_max_distance_label)
        visualization_options_layout.addWidget(rendering_max_distance_spinbox)
        #visualization_options_layout.addStretch(1)
        self.visualization_options_group.setLayout(visualization_options_layout)

    def save_calibration_callback(self):

        output_folder = QFileDialog.getExistingDirectory(
            None, "Select directory to save the calibration result", ".",
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)

        if (output_folder is None or output_folder == ''):
            return

        print(output_folder)

        object_points = np.array(
            self.object_calibration_points + self.external_object_calibration_points,
            dtype=np.float64)
        image_points = np.array(
            self.image_calibration_points + self.external_image_calibration_points,
            dtype=np.float64)

        num_points, dim = object_points.shape
        assert dim == 3
        assert num_points == image_points.shape[0]
        assert 2 == image_points.shape[1]

        np.savetxt(os.path.join(output_folder, "object_points.txt"), object_points)
        np.savetxt(os.path.join(output_folder, "image_points.txt"), image_points)

        if self.optimized_camera_info is not None:

            d = message_to_ordereddict(self.optimized_camera_info)

            with open(os.path.join(output_folder, "optimized_camera_info.json"), 'w') as fout:
                fout.write(json.dumps(d, indent=4, sort_keys=False))

        self.ros_interface.save_calibration_tfs(output_folder)
        pass

    def load_calibration_callback(self):

        input_dir = QFileDialog.getExistingDirectory(
            None, "Select directory to load calibration points from", ".",
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks)

        if (input_dir is None or input_dir == ''):
            return

        print(input_dir)

        object_calibration_points = np.loadtxt(os.path.join(input_dir, "object_points.txt"))
        image_calibration_points = np.loadtxt(os.path.join(input_dir, "image_points.txt"))

        self.object_calibration_points = [p for p in object_calibration_points]
        self.image_calibration_points = [p for p in image_calibration_points]

        print(self.object_calibration_points)
        print(self.image_calibration_points)

        assert len(self.image_calibration_points) == len(self.object_calibration_points)

        self.calibration_possible = \
            len(self.image_calibration_points) + len(self.external_image_calibration_points) \
                >= self.pnp_min_points_spinbox.value()

        self.calibration_button.setEnabled(self.calibration_possible)
        self.calibration2_button.setEnabled(
            self.calibration_possible and
            self.optimize_camera_intrinsics_status and
            not self.optimize_camera_intrinsics_waiting)

        self.calibration_callback()

        self.image_view.set_calibration_points(
            self.object_calibration_points, self.image_calibration_points)
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
        object_calibration_points = \
            self.object_calibration_points + self.external_object_calibration_points
        image_calibration_points = \
            self.image_calibration_points + self.external_image_calibration_points

        if len(object_calibration_points) == 0 or len(image_calibration_points) == 0:
            return

        transform = self.calibrator.calibrate(
            object_calibration_points,
            image_calibration_points)

        if transform is None:
            return

        if self.calibrated_transform is None:
            self.tf_source_combobox.addItem("Calibrator")

        self.calibrated_transform = transform

        self.tf_source_callback(self.tf_source_combobox.currentText())

        self.calibration_api_button.setEnabled(self.calibration_api_request_received
            and self.calibrated_transform is not None)
        self.ros_interface.set_camera_lidar_transform(transform)

    def update_calibration_status(self):

        object_calibration_points = self.object_calibration_points + self.external_object_calibration_points
        image_calibration_points = self.image_calibration_points + self.external_image_calibration_points

        calibration_points_available = len(object_calibration_points) > 0

        if self.calibrated_transform is not None and calibration_points_available:
            calibrated_error, calibrated_inliers = self.calibrator.calculate_reproj_error(
                object_calibration_points,
                image_calibration_points,
                transform_matrix = self.calibrated_transform)
            calibrated_error_string = f"{calibrated_error:.2f}"
            self.calibrated_error = calibrated_error
        else:
            calibrated_error_string = ""
            calibrated_inliers = np.array([])

        if self.source_transform is not None and calibration_points_available:
            source_error, source_inliers = self.calibrator.calculate_reproj_error(
                object_calibration_points,
                image_calibration_points,
                transform_matrix = self.source_transform)
            source_error_string = f"{source_error:.2f}"
        else:
            source_error_string = ""
            source_inliers = np.array([])

        self.calibration_status_points_label.setText(f"#points: {len(object_calibration_points)}")
        self.calibration_status_error_label.setText(
            f"Reproj error: {calibrated_error_string} / {source_error_string}")
        self.calibration_status_inliers_label.setText(
            f"inliers:  {int(calibrated_inliers.sum())} / {int(source_inliers.sum())}")

    def tf_source_callback(self, string):

        string = string.lower()

        if "current" in string:
            assert self.current_transform is not None
            self.source_transform = self.current_transform
        elif "initial" in string:
            assert self.initial_transform is not None
            self.source_transform = self.initial_transform
        elif "calibrator" in string:
            self.source_transform = self.calibrated_transform
        else:
            raise NotImplementedError

        self.update_calibration_status()
        self.image_view.set_transform(self.source_transform)

        self.image_view.update()

    def sensor_data_ros_callback(self, img, camera_info, pointcloud):

        # This method is executed in the ROS spin thread
        with self.lock:
            height, width, channel = img.shape
            bytes_per_line = 3 * width
            q_img = QImage(
                img.data, width, height, bytes_per_line, QImage.Format_RGB888).rgbSwapped()
            self.pixmap_tmp = QPixmap(q_img)

            self.pointcloud_tmp = pointcloud
            self.camera_info_tmp = camera_info

        self.sensor_data_signal.emit()

    def transform_ros_callback(self, transform):

        # This method is executed in the ROS spin thread
        with self.lock:
            self.transform_tmp = transform
            pass

        self.transform_signal.emit()

    def object_point_ros_callback(self, point):

        assert np.prod(point.shape) == 3
        point = point.reshape(3,)
        self.object_point_signal.emit(point[0], point[1], point[2])

    def external_calibration_points_ros_callback(self, object_points, image_points):

        # This method is executed in the ROS spin thread
        with self.lock:
            self.external_object_calibration_points_tmp = object_points
            self.external_image_calibration_points_tmp = image_points

        self.external_calibration_points_signal.emit()

    def optimize_camera_intrinsics_status_callback(self, service_status):
        with self.lock:
            self.optimize_camera_intrinsics_status = service_status
            self.calibration2_button.setEnabled(
                service_status and
                self.calibration_possible and
                not self.optimize_camera_intrinsics_waiting)

    def optimize_camera_intrinsics_result_callback(self, optimized_camera_info):

        with self.lock:
            self.calibration2_button.setEnabled(
                self.calibration_possible and
                self.optimize_camera_intrinsics_status)

            self.calibration2_button.setText("Calibrate intrinsics\n(experimental)")
            self.use_optimized_intrinsics_checkbox.setEnabled(True)

            self.optimized_camera_info_tmp = optimized_camera_info
            self.optimize_camera_intrinsics_waiting = False

            self.optimized_intrinsics_signal.emit()

    def calibration_api_request_received_callback(self):

        self.calibration_api_label.setStyleSheet(
            "QLabel { background-color : yellow; color : black; }")
        self.calibration_api_label.setText("Waiting calibration")

        self.calibration_api_request_received = True

        self.calibration_api_button.setEnabled(self.calibration_api_request_received
            and self.calibrated_transform is not None)

    def calibration_api_request_sent_callback(self):

        self.calibration_api_label.setStyleSheet(
            "QLabel { background-color : green; color : black; }")
        self.calibration_api_label.setText("Calibration sent")

        self.calibration_api_request_received = False
        self.calibration_api_button.setEnabled(False)

    def sensor_data_callback(self):

        # This method is executed in the UI thread
        with self.lock:

            self.image_view.set_pixmap(self.pixmap_tmp)
            self.image_view.set_pointcloud(self.pointcloud_tmp)

            if (
                not( self.use_optimized_intrinsics_checkbox.isChecked() and
                self.use_optimized_intrinsics_checkbox.isEnabled())
            ):
                self.camera_info = self.camera_info_tmp

                self.image_view.set_camera_info(self.camera_info_tmp.k,self.camera_info_tmp.d)
                self.calibrator.set_camera_info(self.camera_info_tmp.k,self.camera_info_tmp.d)

            self.image_view.update()
            self.graphics_view.update()
            pass

    def transform_callback(self):

        # This method is executed in the UI thread
        with self.lock:

            if self.initial_transform is None:
                self.initial_transform = np.copy(self.transform_tmp)
                self.current_transform = self.initial_transform
                self.tf_source_combobox.addItem("Initial /tf")
                self.tf_source_combobox.addItem("Current /tf")

                self.image_view.update()

            changed = (self.transform_tmp != self.current_transform).any()
            self.current_transform = np.copy(self.transform_tmp)

            # the Current /tf case
            if "current" in self.tf_source_combobox.currentText().lower() and changed:
                self.tf_source_callback(self.tf_source_combobox.currentText())

    def image_click_callback(self, x, y):

        p = np.array([x, y]).reshape((2,))

        if self.current_object_point is None:
            self.delete_calibration_points(p)
            return

        self.image_calibration_points.append(p)
        self.object_calibration_points.append(self.current_object_point.reshape(3,))

        self.image_view.set_calibration_points(
            self.object_calibration_points, self.image_calibration_points)

        self.calibration_callback()

        self.current_object_point = None
        self.image_view.set_current_point(None)

        self.calibration_possible = \
            len(self.image_calibration_points) + len(self.external_image_calibration_points) \
                >= self.pnp_min_points_spinbox.value()

        self.calibration_button.setEnabled(self.calibration_possible)
        self.calibration2_button.setEnabled(
            self.calibration_possible and
            self.optimize_camera_intrinsics_status and
            not self.optimize_camera_intrinsics_waiting)

        self.image_view.update()


    def delete_calibration_points(self, p):

        if len(self.object_calibration_points) == 0:
            return

        p = p.reshape(1, 2)

        # Detection deletion feature
        tolerance = 15 # px

        object_points = np.array(self.object_calibration_points)
        image_points = np.array(self.image_calibration_points)

        indexes = np.linalg.norm(image_points - p, axis=1) > tolerance

        object_points = object_points[indexes, :]
        image_points = image_points[indexes, :]

        self.object_calibration_points = [p for p in object_points]
        self.image_calibration_points = [p for p in image_points]

        self.calibration_callback()
        self.image_view.set_calibration_points(self.object_calibration_points,
            self.image_calibration_points)

    def object_point_callback(self, x, y, z):

        point_array = np.array([x, y, z], np.float64).reshape(1,3)

        self.image_view.set_current_point(point_array)
        self.image_view.update()

        self.current_object_point = point_array

    def external_calibration_points_callback(self):

        with self.lock:
            self.external_object_calibration_points = copy.deepcopy(
                self.external_object_calibration_points_tmp)
            self.external_image_calibration_points = copy.deepcopy(
                self.external_image_calibration_points_tmp)

        self.image_view.set_external_calibration_points(
            self.external_object_calibration_points,
            self.external_image_calibration_points_tmp)

        self.calibration_possible = \
            len(self.image_calibration_points) + len(self.external_image_calibration_points) \
                >= self.pnp_min_points_spinbox.value()

        self.calibration_button.setEnabled(self.calibration_possible)
        self.calibration2_button.setEnabled(
            self.calibration_possible and
            self.optimize_camera_intrinsics_status and
            not self.optimize_camera_intrinsics_waiting)

    def optimized_camera_info_callback(self):

        with self.lock:
            self.optimized_camera_info = copy.deepcopy(self.optimized_camera_info_tmp)

def main(args=None):
    app = QApplication(sys.argv)

    rclpy.init(args=args)

    try:
        signal.signal(signal.SIGINT, sigint_handler)

        ros_interface = RosInterface()
        ex = InteractiveCalibratorUI(ros_interface)

        ros_interface.spin()

        sys.exit(app.exec_())
    except (KeyboardInterrupt, SystemExit):
        print("Received sigint. Quiting...")
        rclpy.shutdown()

def sigint_handler(*args):
    QApplication.quit()

if __name__ == '__main__':
    main()
