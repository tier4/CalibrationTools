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

import copy
import threading

from PySide2.QtCore import Qt
from PySide2.QtCore import Signal
from PySide2.QtGui import QImage
from PySide2.QtGui import QPixmap
from PySide2.QtWidgets import QCheckBox
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QDoubleSpinBox
from PySide2.QtWidgets import QGraphicsScene
from PySide2.QtWidgets import QGraphicsView
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QHBoxLayout
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QSpinBox
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
import numpy as np
from tier4_calibration_views.image_view import CustomQGraphicsView
from tier4_calibration_views.image_view import ImageView
from tier4_calibration_views.image_view_ros_interface import ImageViewRosInterface


class ImageViewUI(QMainWindow):
    sensor_data_signal = Signal()
    sensor_data_delay_signal = Signal(float)
    transform_signal = Signal()
    external_calibration_points_signal = Signal()
    optimized_intrinsics_signal = Signal()

    def __init__(self, ros_interface: ImageViewRosInterface):
        super().__init__()
        self.setWindowTitle("Image view (camera-lidar delay=??)")

        # ROS Interface
        self.ros_interface = ros_interface
        self.ros_interface.set_sensor_data_callback(self.sensor_data_ros_callback)
        self.ros_interface.set_sensor_data_delay_callback(self.sensor_data_delay_ros_callback)
        self.ros_interface.set_transform_callback(self.transform_ros_callback)
        self.ros_interface.set_external_calibration_points_callback(
            self.external_calibration_points_ros_callback
        )

        self.sensor_data_signal.connect(self.sensor_data_callback)
        self.sensor_data_delay_signal.connect(self.sensor_data_delay_callback)
        self.transform_signal.connect(self.transform_callback)
        self.external_calibration_points_signal.connect(self.external_calibration_points_callback)

        # Threading variables
        self.lock = threading.RLock()
        self.transform_tmp = None
        self.external_object_calibration_points_tmp = None
        self.external_image_calibration_points_tmp = None
        self.pixmap_tmp = None
        self.camera_info_tmp = None
        self.pointcloud_tmp = None
        self.delay_tmp = None

        # Calibration variables
        self.camera_info = None
        self.initial_transform = None
        self.current_transform = None
        self.calibrated_transform = None
        self.source_transform = None

        # Parent widget
        self.central_widget = QWidget(self)
        self.left_menu_widget = None
        self.right_menu_widget = None

        self.setCentralWidget(self.central_widget)
        self.layout = QHBoxLayout(self.central_widget)

        # Image View
        self.make_image_view()

        # Menu Widgets
        self.make_left_menu()
        self.make_right_menu()

        self.layout.addWidget(self.graphics_view)

        if self.left_menu_widget:
            self.layout.addWidget(self.left_menu_widget)

        if self.right_menu_widget:
            self.layout.addWidget(self.right_menu_widget)

        self.show()

    def make_left_menu(self):
        pass

    def make_right_menu(self):
        self.right_menu_widget = QWidget(self.central_widget)
        self.right_menu_widget.setFixedWidth(210)
        self.right_menu_layout = QVBoxLayout(self.right_menu_widget)

        # Visualization group
        self.make_visualization_options()

        self.right_menu_layout.addWidget(self.visualization_options_group)

    def make_image_view(self):
        self.image_view = ImageView()
        # self.image_view.set_pixmap(pixmap)
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

    def make_calibration_options(self):
        pass

    def make_data_collection_options(self):
        pass

    def make_visualization_options(self):
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
        # visualization_options_layout.addStretch(1)
        self.visualization_options_group.setLayout(visualization_options_layout)

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

        self.image_view.set_transform(self.source_transform)
        self.image_view.update()

    def sensor_data_ros_callback(self, img, camera_info, pointcloud, delay):
        # This method is executed in the ROS spin thread
        with self.lock:
            height, width, _ = img.shape
            bytes_per_line = 3 * width
            q_img = QImage(
                img.data, width, height, bytes_per_line, QImage.Format_RGB888
            ).rgbSwapped()
            self.pixmap_tmp = QPixmap(q_img)

            self.pointcloud_tmp = pointcloud
            self.camera_info_tmp = camera_info
            self.delay_tmp = delay

        self.sensor_data_signal.emit()

    def sensor_data_delay_ros_callback(self, delay):
        with self.lock:
            self.delay_tmp = delay
        self.sensor_data_delay_signal.emit(delay)

    def transform_ros_callback(self, transform):
        # This method is executed in the ROS spin thread
        with self.lock:
            self.transform_tmp = transform

        self.transform_signal.emit()

    def external_calibration_points_ros_callback(self, object_points, image_points):
        # This method is executed in the ROS spin thread
        with self.lock:
            self.external_object_calibration_points_tmp = object_points
            self.external_image_calibration_points_tmp = image_points

        self.external_calibration_points_signal.emit()

    def sensor_data_callback(self):
        # This method is executed in the UI thread
        with self.lock:
            self.image_view.set_pixmap(self.pixmap_tmp)
            self.image_view.set_pointcloud(self.pointcloud_tmp)

            self.camera_info = self.camera_info_tmp
            self.image_view.set_camera_info(self.camera_info_tmp.k, self.camera_info_tmp.d)

            self.image_view.update()
            self.graphics_view.update()

            self.setWindowTitle(
                f"Image view (camera-lidar delay={1000*self.delay_tmp:.2f} ms)"  # noqa E231
            )

    def sensor_data_delay_callback(self, delay):
        # This method is executed in the UI thread
        self.setWindowTitle(
            f"Image view (camera-lidar delay={1000*self.delay_tmp:.2f} ms)"  # noqa E231
        )

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
        pass

    def external_calibration_points_callback(self):
        with self.lock:
            self.external_object_calibration_points = copy.deepcopy(
                self.external_object_calibration_points_tmp
            )
            self.external_image_calibration_points = copy.deepcopy(
                self.external_image_calibration_points_tmp
            )

        self.image_view.set_external_calibration_points(
            self.external_object_calibration_points, self.external_image_calibration_points_tmp
        )
