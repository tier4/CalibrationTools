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
import json
import threading

from PySide2.QtCore import Qt
from PySide2.QtCore import Signal
from PySide2.QtGui import QImage
from PySide2.QtGui import QPixmap
from PySide2.QtWidgets import QCheckBox
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QDoubleSpinBox
from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QGraphicsScene
from PySide2.QtWidgets import QGraphicsView
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QHBoxLayout
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QPlainTextEdit
from PySide2.QtWidgets import QSpinBox
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
import numpy as np
from sensor_msgs.msg import CameraInfo
from tier4_calibration_views.image_view import CustomQGraphicsView
from tier4_calibration_views.image_view import ImageView
from tier4_calibration_views.image_view_ros_interface import ImageViewRosInterface
import transforms3d
import yaml


def load_transform_from_yaml(yaml_data, parent_frame: str, child_frame: str):
    entry = yaml_data[parent_frame][child_frame]

    x = float(entry.get("x"))
    y = float(entry.get("y"))
    z = float(entry.get("z"))

    if all(key in entry for key in ("roll", "pitch", "yaw")):
        roll = float(entry["roll"])
        pitch = float(entry["pitch"])
        yaw = float(entry["yaw"])
        rot = transforms3d.euler.euler2mat(roll, pitch, yaw)
    elif all(key in entry for key in ("qx", "qy", "qz", "qw")):
        qx = float(entry["qx"])
        qy = float(entry["qy"])
        qz = float(entry["qz"])
        qw = float(entry["qw"])
        rot = transforms3d.quaternions.quat2mat((qw, qx, qy, qz))
    else:
        rot = np.eye(3)

    mat = np.eye(4)
    mat[0:3, 0:3] = rot
    mat[0:3, 3] = [x, y, z]

    return mat


def load_transform_from_json(json_data, parent_frame: str, child_frame: str):
    if (
        json_data["header"]["frame_id"] != parent_frame
        or json_data["child_frame_id"] != child_frame
    ):
        raise KeyError(
            f"Expected transform from {parent_frame} to {child_frame}, but got from {json_data['header']['frame_id']} to {json_data['child_frame_id']}"
        )

    translation = json_data["transform"]["translation"]
    rotation = json_data["transform"]["rotation"]

    x = translation["x"]
    y = translation["y"]
    z = translation["z"]

    qx = rotation["x"]
    qy = rotation["y"]
    qz = rotation["z"]
    qw = rotation["w"]
    rot = transforms3d.quaternions.quat2mat((qw, qx, qy, qz))

    mat = np.eye(4)
    mat[0:3, 0:3] = rot
    mat[0:3, 3] = [x, y, z]

    return mat


def load_camera_info_from_yaml(yaml_data) -> CameraInfo:
    camera_info = CameraInfo()
    camera_info.width = yaml_data["image_width"]
    camera_info.height = yaml_data["image_height"]
    camera_info.distortion_model = yaml_data["distortion_model"]

    camera_info.d = yaml_data["distortion_coefficients"]["data"]
    camera_info.k = yaml_data["camera_matrix"]["data"]
    camera_info.p = yaml_data["projection_matrix"]["data"]
    camera_info.r = yaml_data["rectification_matrix"]["data"]

    return camera_info


def load_camera_info_from_json(json_data) -> CameraInfo:
    camera_info = CameraInfo()

    camera_info.header.frame_id = json_data["header"]["frame_id"]
    camera_info.width = json_data["width"]
    camera_info.height = json_data["height"]
    camera_info.distortion_model = json_data["distortion_model"]
    camera_info.d = json_data["d"]
    camera_info.k = json_data["k"]
    camera_info.p = json_data["p"]
    camera_info.r = json_data["r"]

    return camera_info


def validate_camera_info(camera_info: CameraInfo) -> bool:
    if camera_info is None:
        return False

    if len(camera_info.k) != 9:
        return False
    if camera_info.k[0] <= 0 or camera_info.k[4] <= 0:
        # fx, fy should be positive
        return False
    if (
        camera_info.k[1] != 0
        or camera_info.k[3] != 0
        or camera_info.k[6] != 0
        or camera_info.k[7] != 0
    ):
        # skew and other parameters should be zero
        return False
    if len(camera_info.d) not in [4, 5, 8, 12, 14]:
        return False
    if len(camera_info.r) != 9:
        return False
    if len(camera_info.p) != 12:
        return False
    if camera_info.distortion_model not in [
        "plumb_bob",
        "rational_polynomial",
    ]:
        return False
    if camera_info.width <= 0 or camera_info.height <= 0:
        return False
    return True


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
        self.message_camera_info_tmp = None
        self.pointcloud_tmp = None
        self.delay_tmp = None

        # Calibration variables
        self.message_camera_info = None
        self.optimized_camera_info = None
        self.source_camera_info = None

        self.initial_transform = None
        self.current_transform = None
        self.calibrated_transform = None
        self.source_transform = None

        # Parent widget
        self.central_widget = QWidget(self)
        self.left_menu_widget = None
        self.right_menu_widget_1 = None
        self.right_menu_widget_2 = None

        self.setCentralWidget(self.central_widget)
        self.layout = QHBoxLayout(self.central_widget)

        # Image View
        self.make_image_view()

        # Menu Widgets
        self.make_left_menu()
        self.make_right_menu_1()
        self.make_right_menu_2()

        self.layout.addWidget(self.graphics_view)

        if self.left_menu_widget:
            self.layout.addWidget(self.left_menu_widget)

        if self.right_menu_widget_1:
            self.layout.addWidget(self.right_menu_widget_1)
        if self.right_menu_widget_2:
            self.layout.addWidget(self.right_menu_widget_2)

        self.show()

    def make_left_menu(self):
        pass

    def make_right_menu_1(self):
        self.right_menu_widget_1 = QWidget(self.central_widget)
        self.right_menu_widget_1.setFixedWidth(210)
        self.right_menu_layout_1 = QVBoxLayout(self.right_menu_widget_1)

        # Visualization group
        self.make_visualization_options()

        self.right_menu_layout_1.addWidget(self.visualization_options_group)

    def make_right_menu_2(self):
        self.right_menu_widget_2 = QWidget(self.central_widget)
        self.right_menu_widget_2.setFixedWidth(210)
        self.right_menu_layout_2 = QVBoxLayout(self.right_menu_widget_2)

        # Source group
        self.make_source_options()

        self.right_menu_layout_2.addWidget(self.source_options_group)

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
        rendering_min_distance_spinbox.setRange(0.01, 200.0)
        rendering_min_distance_spinbox.setSingleStep(0.1)
        rendering_min_distance_spinbox.setValue(0.1)

        def rendering_max_distance_callback(value):
            self.image_view.set_max_rendering_distance(value)

        rendering_max_distance_label = QLabel("Max rendering distance (m)")
        rendering_max_distance_spinbox = QDoubleSpinBox()
        rendering_max_distance_spinbox.valueChanged.connect(rendering_max_distance_callback)
        rendering_max_distance_spinbox.setRange(0.01, 200.0)
        rendering_max_distance_spinbox.setSingleStep(0.1)
        rendering_max_distance_spinbox.setValue(100.0)

        def render_pointcloud_callback(value):
            self.image_view.set_draw_pointcloud(value == Qt.Checked)

        render_pointcloud_checkbox = QCheckBox("Show pointcloud")
        render_pointcloud_checkbox.stateChanged.connect(render_pointcloud_callback)
        render_pointcloud_checkbox.setChecked(True)

        def render_calibration_points_callback(value):
            self.image_view.set_draw_calibration_points(value == Qt.Checked)

        render_calibration_points_checkbox = QCheckBox("Show calibration pairs")
        render_calibration_points_checkbox.stateChanged.connect(render_calibration_points_callback)
        render_calibration_points_checkbox.setChecked(True)

        def render_inliers_callback(value):
            self.image_view.set_draw_inliers(value == Qt.Checked)

        self.render_inliers_checkbox = QCheckBox("Show inliers")
        self.render_inliers_checkbox.stateChanged.connect(render_inliers_callback)
        self.render_inliers_checkbox.setChecked(False)
        self.render_inliers_checkbox.setEnabled(False)

        visualization_options_layout = QVBoxLayout()
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

    def make_source_options(self):
        self.source_options_group = QGroupBox("Source options")
        self.source_options_group.setFlat(True)

        tf_source_label = QLabel("TF source:")
        self.tf_source_combobox = QComboBox()
        self.tf_source_combobox.addItem("Load from File", "file")
        self.tf_source_combobox.setCurrentIndex(-1)  # do not select "file" by default

        def tf_source_index_callback(index):
            if index != -1:
                self.tf_source_callback(self.tf_source_combobox.itemData(index))

        # `activated` is favored over `currentTextChanged`;
        # selecting "file" again should show file dialog regardless of the current selection
        self.tf_source_combobox.activated.connect(tf_source_index_callback)

        self.tf_source_status_text = QPlainTextEdit()
        self.tf_source_status_text.setReadOnly(True)
        self.tf_source_status_text.setPlainText("TFs not available")

        camera_info_source_label = QLabel("Camera info source:")
        self.camera_info_source_combobox = QComboBox()
        self.camera_info_source_combobox.addItem("ROS topic", "message")
        self.camera_info_source_combobox.addItem("Load from File", "file")
        self.camera_info_source_combobox.setCurrentIndex(0)

        def camera_info_source_index_callback(index):
            if index != -1:
                self.camera_info_source_callback(self.camera_info_source_combobox.itemData(index))

        self.camera_info_source_combobox.activated.connect(camera_info_source_index_callback)

        self.camera_info_source_status_text = QPlainTextEdit()
        self.camera_info_source_status_text.setReadOnly(True)
        self.camera_info_source_status_text.setPlainText("Camera info not available")

        source_options_layout = QVBoxLayout()
        source_options_layout.addWidget(tf_source_label)
        source_options_layout.addWidget(self.tf_source_combobox)
        source_options_layout.addWidget(self.tf_source_status_text)
        source_options_layout.addWidget(camera_info_source_label)
        source_options_layout.addWidget(self.camera_info_source_combobox)
        source_options_layout.addWidget(self.camera_info_source_status_text)

        self.source_options_group.setLayout(source_options_layout)

    def tf_source_callback(self, source):
        if source == "current":
            assert self.current_transform is not None
            self.source_transform = self.current_transform
        elif source == "initial":
            assert self.initial_transform is not None
            self.source_transform = self.initial_transform
        elif source == "calibrator":
            assert self.calibrated_transform is not None
            self.source_transform = self.calibrated_transform
        elif source == "file":
            # Reads TF regardless of the format:
            # - yaml or json (output from previous versions)
            # - xyz + quaternion or xyz + rpy
            # - preprocessed or non-preprocessed entry
            filename, _ = QFileDialog.getOpenFileName(
                self, "Open TF File", ".", "YAML file (*.yaml);;JSON file (*.json)"
            )
            if len(filename) == 0:
                return
            try:
                file_to_data, load_transform = (
                    (json.load, load_transform_from_json)
                    if filename.endswith(".json")
                    else (yaml.safe_load, load_transform_from_yaml)
                )

                with open(filename, "r") as f:
                    data = file_to_data(f)

                    try:
                        mat = load_transform(
                            data, self.ros_interface.parent_frame, self.ros_interface.child_frame
                        )
                        self.source_transform = self.ros_interface.get_image_to_lidar_transform(mat)
                    except KeyError:
                        # Fallback to non-postprocessed entry.
                        # If this fails, silently return.
                        mat = load_transform(
                            data, self.ros_interface.image_frame, self.ros_interface.lidar_frame
                        )
                        self.source_transform = mat
            except Exception as ex:
                self.ros_interface.get_logger().error(
                    f"Could not load valid TF from {filename}. {ex}"
                )
                return

        else:
            raise NotImplementedError

        self.image_view.set_transform(self.source_transform)
        self.image_view.update()

        # update status text
        source_xyz = self.source_transform[0:3, 3]
        source_rpy = transforms3d.euler.mat2euler(self.source_transform[0:3, 0:3])
        status_text = (
            f"{self.ros_interface.image_frame}\n"
            f"-> {self.ros_interface.lidar_frame}:\n"
            f"x: {round(source_xyz[0], 6)}\n"
            f"y: {round(source_xyz[1], 6)}\n"
            f"z: {round(source_xyz[2], 6)}\n"
            f"roll: {round(source_rpy[0], 6)}\n"
            f"pitch: {round(source_rpy[1], 6)}\n"
            f"yaw: {round(source_rpy[2], 6)}\n"
        )
        self.tf_source_status_text.setPlainText(status_text)

        # postprocess the source transform
        postprocessed_transform = self.ros_interface.get_parent_to_child_transform(
            self.source_transform
        )
        if postprocessed_transform is None:
            return

        # update (append) status text
        postprocessed_xyz = postprocessed_transform[0:3, 3]
        postprocessed_rpy = transforms3d.euler.mat2euler(postprocessed_transform[0:3, 0:3])
        status_text += (
            f"\n{self.ros_interface.parent_frame}\n"
            f"-> {self.ros_interface.child_frame}:\n"
            f"x: {round(postprocessed_xyz[0], 6)}\n"
            f"y: {round(postprocessed_xyz[1], 6)}\n"
            f"z: {round(postprocessed_xyz[2], 6)}\n"
            f"roll: {round(postprocessed_rpy[0], 6)}\n"
            f"pitch: {round(postprocessed_rpy[1], 6)}\n"
            f"yaw: {round(postprocessed_rpy[2], 6)}\n"
        )
        self.tf_source_status_text.setPlainText(status_text)

    def camera_info_source_callback(self, source):
        if source == "message":
            if not validate_camera_info(self.message_camera_info):
                return
            self.source_camera_info = self.message_camera_info
        elif source == "calibrator":
            if not validate_camera_info(self.optimized_camera_info):
                return
            self.source_camera_info = self.optimized_camera_info
        elif source == "file":
            # Reads TF regardless of the format:
            # - yaml or json (output from previous versions)
            # - xyz + quaternion or xyz + rpy
            # - preprocessed or non-preprocessed entry
            filename, _ = QFileDialog.getOpenFileName(
                self, "Open camera info File", ".", "YAML file (*.yaml);;JSON file (*.json)"
            )
            if len(filename) == 0:
                return
            try:
                file_to_data, load_camera_info = (
                    (json.load, load_camera_info_from_json)
                    if filename.endswith(".json")
                    else (yaml.safe_load, load_camera_info_from_yaml)
                )

                with open(filename, "r") as f:
                    data = file_to_data(f)
                    camera_info = load_camera_info(data)

                    if not validate_camera_info(camera_info):
                        raise ValueError("Invalid CameraInfo data")

                    self.source_camera_info = camera_info
            except Exception as ex:
                self.ros_interface.get_logger().error(
                    f"Could not load valid CameraInfo from {filename}. {ex}"
                )
                return
        else:
            raise NotImplementedError

        self.image_view.set_camera_info(self.source_camera_info.k, self.source_camera_info.d)
        self.image_view.update()
        self.graphics_view.update()

        # update status text
        K = self.source_camera_info.k
        D_str = ", ".join([f"{round(p, 6)}" for p in self.source_camera_info.d])
        status_text = (
            f"D: [{D_str}]\n"
            f"K: [\n"
            f"  {round(K[0], 6)}, {round(K[1], 6)}, {round(K[2], 6)},\n"
            f"  {round(K[3], 6)}, {round(K[4], 6)}, {round(K[5], 6)},\n"
            f"  {round(K[6], 6)}, {round(K[7], 6)}, {round(K[8], 6)},\n"
            f"]\n"
        )
        self.camera_info_source_status_text.setPlainText(status_text)

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
            self.message_camera_info_tmp = camera_info
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

            self.message_camera_info = self.message_camera_info_tmp

            # Force update on the message source case
            source = self.camera_info_source_combobox.currentData()
            if "message" == source:
                self.camera_info_source_callback(source)

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

                self.tf_source_combobox.addItem("ROS topic (initial)", "initial")
                self.tf_source_combobox.addItem("ROS topic (current)", "current")

                self.image_view.update()

            is_changed = (self.transform_tmp != self.current_transform).any()
            self.current_transform = np.copy(self.transform_tmp)

            # if not selected, switch to "initial" and force update
            if self.tf_source_combobox.currentIndex() == -1:
                self.tf_source_combobox.setCurrentIndex(self.tf_source_combobox.findData("initial"))
                self.tf_source_callback("initial")

            # Force update on the current /tf case
            source = self.tf_source_combobox.currentData()
            if "current" == source and is_changed:
                self.tf_source_callback(source)

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
