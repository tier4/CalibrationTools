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

from collections import defaultdict
import copy
import signal
import sys
import threading
from typing import Dict

from PySide2.QtCore import QThread
from PySide2.QtCore import Qt
from PySide2.QtCore import Signal
from PySide2.QtGui import QColor
from PySide2.QtWidgets import QApplication
from PySide2.QtWidgets import QCheckBox
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QDoubleSpinBox
from PySide2.QtWidgets import QGraphicsScene
from PySide2.QtWidgets import QGraphicsView
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QHBoxLayout
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QMainWindow
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
import cv2

# import debugpy
from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.board_detectors.board_detector_factory import make_detector
from intrinsic_camera_calibrator.boards import BoardEnum
from intrinsic_camera_calibrator.boards import BoardParameters
from intrinsic_camera_calibrator.calibrators.calibrator import Calibrator
from intrinsic_camera_calibrator.calibrators.calibrator import CalibratorEnum
from intrinsic_camera_calibrator.calibrators.calibrator_factory import make_calibrator
from intrinsic_camera_calibrator.camera_model import CameraModel
from intrinsic_camera_calibrator.data_collector import CollectionStatus
from intrinsic_camera_calibrator.data_collector import DataCollector
from intrinsic_camera_calibrator.data_sources.data_source import DataSource
from intrinsic_camera_calibrator.types import ImageViewMode
from intrinsic_camera_calibrator.types import OperationMode
from intrinsic_camera_calibrator.views.data_collector_view import DataCollectorView
from intrinsic_camera_calibrator.views.image_view import CustomQGraphicsView
from intrinsic_camera_calibrator.views.image_view import ImageView
from intrinsic_camera_calibrator.views.initialization_view import InitializationView
from intrinsic_camera_calibrator.views.parameter_view import ParameterView

# from rosidl_runtime_py.convert import message_to_ordereddict
# from intrinsic_camera_calibrator.ros_interface import RosInterface
import numpy as np
import rclpy


class CameraIntrinsicsCalibratorUI(QMainWindow):

    produced_data_signal = Signal()
    consumed_data_signal = Signal()
    should_process_image = Signal()
    request_image_detection = Signal(object)

    def __init__(self, cfg):
        super().__init__()
        self.setWindowTitle("Camera intrinsics calibrator")

        self.cfg = defaultdict(dict, cfg)

        # Threading variables
        self.lock = threading.RLock()
        self.unprocessed_image = None
        self.pending_detection_request = False
        self.pending_detection_result = False

        self.detector_thread = QThread()
        self.detector_thread.start()

        self.calibration_thread = QThread()
        self.calibration_thread.start()

        self.operation_mode = OperationMode.IDLE
        self.board_type = BoardEnum.CHESSBOARD
        self.board_parameters = BoardParameters(lock=self.lock, cfg=self.cfg["board_parameters"])
        self.detector: BoardDetector = None
        self.data_collector = DataCollector()
        self.calibrator_dict: Dict[CalibratorEnum, Calibrator] = {}

        for calibrator_type in CalibratorEnum:
            calibrator_cfg = defaultdict()

            if (
                "calibrator_type" in self.cfg
                and calibrator_type.value["name"] == self.cfg["calibrator_type"]
            ):
                calibrator_cfg = self.cfg["calibration_parameters"]

            calibrator = make_calibrator(calibrator_type, lock=self.lock, cfg=calibrator_cfg)
            self.calibrator_dict[calibrator_type] = calibrator

            calibrator.moveToThread(self.calibration_thread)
            calibrator.calibration_results_signal.connect(self.process_calibration_results)

        self.image_view_mode = ImageViewMode.SOURCE_UNRECTIFIED
        self.paused = False

        self.img_id = 0
        self.should_process_image.connect(self.process_data)  # , Qt.QueuedConnection
        self.produced_data_signal.connect(self.process_new_data)

        self.detector_dict = {}

        # Parent widget
        self.central_widget = QWidget(self)
        # self.central_widget.resize(1000,1000)

        self.setCentralWidget(self.central_widget)
        self.layout = QHBoxLayout(self.central_widget)

        # Image View
        self.make_image_view()

        # Menu Widgets
        self.left_menu_widget = QWidget(self.central_widget)
        self.left_menu_widget.setFixedWidth(300)
        self.left_menu_layout = QVBoxLayout(self.left_menu_widget)

        self.right_menu_widget = QWidget(self.central_widget)
        self.right_menu_widget.setFixedWidth(300)
        self.right_menu_layout = QVBoxLayout(self.right_menu_widget)

        # Mode group
        self.make_mode_group()

        # Calibration group
        self.make_calibration_group()

        # Detector group
        self.make_detector_group()

        # Detections group
        self.make_detection_group()

        # Data collection group
        self.make_data_collection_group()

        # Visualization group
        self.make_visualization_group()

        # self.menu_layout.addWidget(label)
        self.left_menu_layout.addWidget(self.calibration_group)
        self.left_menu_layout.addWidget(self.detector_options_group)
        self.left_menu_layout.addWidget(self.raw_detection_results_group)
        self.left_menu_layout.addWidget(self.single_shot_detection_results_group)
        # self.left_menu_layout.addWidget(self.calibration_status_group)
        # self.left_menu_layout.addWidget(self.calibration_options_group)

        self.right_menu_layout.addWidget(self.mode_options_group)
        self.right_menu_layout.addWidget(self.data_collection_group)
        self.right_menu_layout.addWidget(self.visualization_options_group)
        # self.right_menu_layout.addWidget(self.visualization_options_group)

        self.layout.addWidget(self.graphics_view)
        # self.layout.addWidget(self.image_view)

        self.layout.addWidget(self.left_menu_widget)
        self.layout.addWidget(self.right_menu_widget)

        self.show()
        self.setEnabled(False)

        self.initialization_view = InitializationView(self, cfg)

    def make_image_view(self):

        self.image_view = ImageView()
        # self.image_view.set_pixmap(pixmap)
        # self.image_view.clicked_signal.connect(self.image_click_callback)

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

    def make_mode_group(self):
        self.mode_options_group = QGroupBox("Mode options")
        self.mode_options_group.setFlat(True)

        self.pause_button = QPushButton("Pause")

        def pause_callback():
            if self.paused:
                self.pause_button.setText("Pause")
                self.paused = False
            else:
                self.pause_button.setText("Resume")
                self.paused = True
                self.should_process_image.emit()

        self.pause_button.clicked.connect(pause_callback)

        mode_options_layout = QVBoxLayout()
        mode_options_layout.addWidget(self.pause_button)
        self.mode_options_group.setLayout(mode_options_layout)

    def make_calibration_group(self):

        self.calibration_group = QGroupBox("Calibration options")
        self.calibration_group.setFlat(True)

        self.calibrator_type_combobox = QComboBox()
        self.calibration_parameters_button = QPushButton(
            "Calibration parameters"
        )  # this view should disable the previous combobox
        self.calibration_button = QPushButton("Calibrate")
        self.calibration_status_label = QLabel("Calibration status: idle")
        self.calibration_time_label = QLabel("Calibration time:")
        self.calibration_training_samples_label = QLabel("Training samples:")
        self.calibration_pre_rejection_inliers_label = QLabel("Pre rejection inliers:")
        self.calibration_post_rejection_inliers_label = QLabel("Post rejection inliers:")
        self.calibration_evaluation_samples_label = QLabel("Evaluation samples:")

        self.calibration_training_rms_label = QLabel("Training (all) rms error:")
        self.calibration_inlier_rms_label = QLabel("Training (inlier) rms error:")
        self.calibration_evaluation_rms_label = QLabel("Evaluation (all) rms error:")

        def on_parameters_view_closed():
            self.calibrator_type_combobox.setEnabled(True)
            self.calibration_parameters_button.setEnabled(True)

        def on_parameters_button_clicked():
            self.calibrator_type_combobox.setEnabled(False)
            self.calibration_parameters_button.setEnabled(False)
            calibrator_type = self.calibrator_type_combobox.currentData()

            data_collection_parameters_view = ParameterView(self.calibrator_dict[calibrator_type])
            data_collection_parameters_view.closed.connect(on_parameters_view_closed)

        def on_calibration_clicked():
            calibrator_type = self.calibrator_type_combobox.currentData()
            self.calibrator_dict[calibrator_type].calibration_request.emit(
                self.data_collector.clone_without_images()
            )

            self.calibrator_type_combobox.setEnabled(False)
            self.calibration_parameters_button.setEnabled(False)
            self.calibration_button.setEnabled(False)

            self.calibration_status_label.setText("Calibration status: calibrating")

        self.calibration_parameters_button.clicked.connect(on_parameters_button_clicked)
        self.calibration_button.clicked.connect(on_calibration_clicked)

        for calibrator_type in CalibratorEnum:
            self.calibrator_type_combobox.addItem(calibrator_type.value["display"], calibrator_type)

        if "calibrator_type" in self.cfg:
            try:
                self.calibrator_type_combobox.setCurrentIndex(
                    CalibratorEnum.from_name(self.cfg["calibrator_type"]).get_id()
                )
            except Exception as e:
                print(f"Invalid calibration_type: {e}")
        else:
            self.calibrator_type_combobox.setCurrentIndex(0)

        calibration_layout = QVBoxLayout()
        calibration_layout.addWidget(self.calibrator_type_combobox)
        calibration_layout.addWidget(self.calibration_parameters_button)
        calibration_layout.addWidget(self.calibration_button)
        calibration_layout.addWidget(self.calibration_status_label)
        calibration_layout.addWidget(self.calibration_time_label)
        calibration_layout.addWidget(self.calibration_training_samples_label)
        calibration_layout.addWidget(self.calibration_pre_rejection_inliers_label)
        calibration_layout.addWidget(self.calibration_post_rejection_inliers_label)
        calibration_layout.addWidget(self.calibration_evaluation_samples_label)
        calibration_layout.addWidget(self.calibration_training_rms_label)
        calibration_layout.addWidget(self.calibration_inlier_rms_label)
        calibration_layout.addWidget(self.calibration_evaluation_rms_label)

        self.calibration_group.setLayout(calibration_layout)

    def make_detector_group(self):
        def detector_parameters_button_callback():
            print("detector_parameters_button_callback")
            self.detector_parameters_view = ParameterView(self.detector)
            self.detector_parameters_view.parameter_changed.connect(self.on_parameter_changed)
            self.board_type_combobox.setEnabled(False)

        self.detector_options_group = QGroupBox("Detection options")
        self.detector_options_group.setFlat(True)

        self.detector_parameters_button = QPushButton("Detector parameters")

        self.detector_parameters_button.clicked.connect(detector_parameters_button_callback)

        detector_options_layout = QVBoxLayout()
        detector_options_layout.addWidget(self.detector_parameters_button)
        self.detector_options_group.setLayout(detector_options_layout)

    def make_detection_group(self):

        self.raw_detection_results_group = QGroupBox("Detection results")
        self.raw_detection_results_group.setFlat(True)

        self.single_shot_detection_results_group = QGroupBox(
            "Single-shot calibration detection results"
        )
        self.single_shot_detection_results_group.setFlat(True)

        self.raw_detection_label = QLabel("Detected:")
        self.raw_linear_error_rms_label = QLabel("Linear error (rms):")
        self.rough_tilt_label = QLabel("Rough tilt:")
        self.rough_angles_label = QLabel("Rough angles:")
        self.rough_position_label = QLabel("Rough position:")
        self.skew_label = QLabel("Skew:")
        self.relative_area_label = QLabel("Relative area:")

        self.single_shot_linear_error_rms_label = QLabel("Linear error (rms):")
        self.single_shot_reproj_error_max_label = QLabel("Reproj error (max):")
        self.single_shot_reproj_error_avg_label = QLabel("Reproj error (avg):")
        self.single_shot_reproj_error_rms_label = QLabel("Reproj error (rms):")

        raw_detection_results_layout = QVBoxLayout()
        single_shot_detection_results_layout = QVBoxLayout()

        raw_detection_results_layout.addWidget(self.raw_detection_label)
        raw_detection_results_layout.addWidget(self.rough_tilt_label)
        raw_detection_results_layout.addWidget(self.rough_angles_label)
        raw_detection_results_layout.addWidget(self.rough_position_label)
        raw_detection_results_layout.addWidget(self.skew_label)
        raw_detection_results_layout.addWidget(self.relative_area_label)
        raw_detection_results_layout.addWidget(self.raw_linear_error_rms_label)

        single_shot_detection_results_layout.addWidget(self.single_shot_linear_error_rms_label)
        single_shot_detection_results_layout.addWidget(self.single_shot_reproj_error_max_label)
        single_shot_detection_results_layout.addWidget(self.single_shot_reproj_error_avg_label)
        single_shot_detection_results_layout.addWidget(self.single_shot_reproj_error_rms_label)

        self.raw_detection_results_group.setLayout(raw_detection_results_layout)
        self.single_shot_detection_results_group.setLayout(single_shot_detection_results_layout)

    def make_data_collection_group(self):

        self.data_collection_group = QGroupBox("Data collection")
        self.data_collection_group.setFlat(True)

        self.data_collection_training_label = QLabel("Training samples:")
        self.data_collection_evaluation_label = QLabel("Evaluation samples:")
        self.training_occupancy_rate_label = QLabel("Training occupancy:")
        self.evaluation_occupancy_rate_label = QLabel("Evaluation occupancy:")

        def view_data_collection_statistics_callback():

            print("view_data_collection_statistics_callback")  # here we whould
            data_collection_statistics_view = DataCollectorView(
                self.data_collector.clone_without_images()
            )
            data_collection_statistics_view.plot()

        def data_collection_parameters_closed_callback():
            self.data_collection_parameters_button.setEnabled(True)

        def data_collection_parameters_callback():
            self.data_collection_parameters_button.setEnabled(False)

            print("data_collection_parameters_callback")  # here we whould
            data_collection_parameters_view = ParameterView(self.data_collector)
            data_collection_parameters_view.closed.connect(
                data_collection_parameters_closed_callback
            )

        self.view_data_collection_statistics_button = QPushButton("View data collection statistics")
        self.view_data_collection_statistics_button.clicked.connect(
            view_data_collection_statistics_callback
        )

        self.data_collection_parameters_button = QPushButton("Data collection parameters")
        self.data_collection_parameters_button.clicked.connect(data_collection_parameters_callback)

        data_collection_layout = QVBoxLayout()

        data_collection_layout.addWidget(self.data_collection_training_label)
        data_collection_layout.addWidget(self.data_collection_evaluation_label)
        data_collection_layout.addWidget(self.training_occupancy_rate_label)
        data_collection_layout.addWidget(self.evaluation_occupancy_rate_label)

        data_collection_layout.addWidget(self.view_data_collection_statistics_button)
        data_collection_layout.addWidget(self.data_collection_parameters_button)

        self.data_collection_group.setLayout(data_collection_layout)

    def make_visualization_group(self):

        self.visualization_options_group = QGroupBox("Visualization options")
        self.visualization_options_group.setFlat(True)

        def draw_detection_checkbox_callback(value):
            self.image_view.set_draw_detection_points(value == Qt.Checked)
            self.should_process_image.emit()

        draw_detection_checkbox = QCheckBox("Draw detection")
        draw_detection_checkbox.setChecked(True)
        draw_detection_checkbox.stateChanged.connect(draw_detection_checkbox_callback)

        def draw_training_points_checkbox_callback(value):
            self.image_view.set_draw_training_points(value == Qt.Checked)
            self.should_process_image.emit()

        self.draw_training_points_checkbox = QCheckBox("Draw training points")
        self.draw_training_points_checkbox.setChecked(False)
        self.draw_training_points_checkbox.stateChanged.connect(
            draw_training_points_checkbox_callback
        )

        def draw_evaluation_points_checkbox_callback(value):
            self.image_view.set_draw_evaluation_points(value == Qt.Checked)
            self.should_process_image.emit()

        self.draw_evaluation_points_checkbox = QCheckBox("Draw evaluation points")
        self.draw_evaluation_points_checkbox.setChecked(False)
        self.draw_evaluation_points_checkbox.stateChanged.connect(
            draw_evaluation_points_checkbox_callback
        )

        def draw_training_heatmap_callback(value):
            self.image_view.set_draw_training_heatmap(value == Qt.Checked)
            self.should_process_image.emit()

        self.draw_training_heatmap_checkbox = QCheckBox("Draw training occupancy")
        self.draw_training_heatmap_checkbox.setChecked(False)
        self.draw_training_heatmap_checkbox.stateChanged.connect(draw_training_heatmap_callback)

        def draw_evaluation_heatmap_callback(value):
            self.image_view.set_draw_evaluation_heatmap(value == Qt.Checked)
            self.should_process_image.emit()

        self.draw_evaluation_heatmap_checkbox = QCheckBox("Draw evaluation occupancy")
        self.draw_evaluation_heatmap_checkbox.setChecked(False)
        self.draw_evaluation_heatmap_checkbox.stateChanged.connect(draw_evaluation_heatmap_callback)

        rendering_alpha_label = QLabel("Drawings alpha:")

        self.rendering_alpha_spinbox = QDoubleSpinBox()
        self.rendering_alpha_spinbox.setDecimals(2)
        self.rendering_alpha_spinbox.setRange(0.0, 1.0)
        self.rendering_alpha_spinbox.setSingleStep(0.05)
        self.rendering_alpha_spinbox.setValue(1.0)
        self.rendering_alpha_spinbox.valueChanged.connect(lambda: self.should_process_image.emit())

        visualization_options_layout = QVBoxLayout()
        visualization_options_layout.addWidget(draw_detection_checkbox)
        visualization_options_layout.addWidget(self.draw_training_points_checkbox)
        visualization_options_layout.addWidget(self.draw_evaluation_points_checkbox)
        visualization_options_layout.addWidget(self.draw_training_heatmap_checkbox)
        visualization_options_layout.addWidget(self.draw_evaluation_heatmap_checkbox)
        visualization_options_layout.addWidget(rendering_alpha_label)
        visualization_options_layout.addWidget(self.rendering_alpha_spinbox)
        self.visualization_options_group.setLayout(visualization_options_layout)

    def start(self, mode: OperationMode, data_source: DataSource, board_type: BoardEnum):
        self.operation_mode = mode
        self.data_source = data_source
        self.board_type = board_type
        self.setEnabled(True)

        print("Init")
        print(f"mode : {mode}")
        print(f"data_source : {data_source}")
        print(f"board_type : {board_type}")

        detector_cfg = self.cfg[self.board_type.value["name"] + "_detector"]

        self.detector = make_detector(
            board_type=self.board_type,
            lock=self.lock,
            board_parameters=self.board_parameters,
            cfg=detector_cfg,
        )

        self.detector.moveToThread(self.detector_thread)
        self.detector.detection_results_signal.connect(self.process_detection_results)
        self.request_image_detection.connect(self.detector.detect)

    def process_calibration_results(
        self,
        calibrated_model: CameraModel,
        dt: float,
        num_training_detections: int,
        num_pre_rejection_inliers: int,
        num_post_rejection_inliers: int,
        num_evaluation_detections: int,
        training_rms_error: float,
        inlier_rms_error: float,
        evaluation_rms_error: float,
    ):
        self.calibration_status_label.setText("Calibration status: idle")
        self.calibration_time_label.setText(f"Calibration time: {dt:.2f}s")
        self.calibration_training_samples_label.setText(
            f"Training samples: {num_training_detections}"
        )
        self.calibration_pre_rejection_inliers_label.setText(
            f"Pre rejection inliers: {num_pre_rejection_inliers}"
        )
        self.calibration_post_rejection_inliers_label.setText(
            f"Post rejection inliers: {num_post_rejection_inliers}"
        )
        self.calibration_evaluation_samples_label.setText(
            f"Evaluation samples: {num_evaluation_detections}"
        )

        self.calibration_training_rms_label.setText(
            f"Training (all) rms error: {training_rms_error:.2f}"
        )
        self.calibration_inlier_rms_label.setText(
            f"Training (inlier) rms error: {inlier_rms_error:.2f}"
        )
        self.calibration_evaluation_rms_label.setText(
            f"Evaluation (all) rms error: {evaluation_rms_error:.2f}"
        )

        self.calibrator_type_combobox.setEnabled(True)
        self.calibration_parameters_button.setEnabled(True)
        self.calibration_button.setEnabled(True)

    def process_detection_results(self, img: np.array, detection: BoardDetection):
        if img is None:
            self.pending_detection_result = False

            if self.pending_detection_request:
                self.should_process_image.emit()

            return

        # print(f"{threading.get_ident()} -> process_detection_results: start")
        cv2.putText(
            img,
            f"image id = {self.img_id}",
            (100, 100),
            cv2.FONT_HERSHEY_COMPLEX,
            5,
            (255, 0, 255),
            1,
            cv2.LINE_AA,
        )

        # Set general options
        self.image_view.set_rendering_alpha(self.rendering_alpha_spinbox.value())

        # Set detection drawings
        if detection is None:
            self.image_view.set_detection_ordered_points(None)

            self.raw_detection_label.setText("Detected: False")
            self.raw_linear_error_rms_label.setText("Linear error rms:")
            self.rough_tilt_label.setText("Rough tilt:")
            self.rough_angles_label.setText("Rough angles:")
            self.rough_position_label.setText("Rough position:")
            self.skew_label.setText("Skew:")
            self.relative_area_label.setText("Relative area:")

            self.single_shot_linear_error_rms_label.setText("Linear error (rms):")
            self.single_shot_reproj_error_max_label.setText("Reproj error (max):")
            self.single_shot_reproj_error_avg_label.setText("Reproj error (avg):")
            self.single_shot_reproj_error_rms_label.setText("Reproj error (rms):")

        elif self.board_type == BoardEnum.CHESSBOARD or self.board_type == BoardEnum.DOTBOARD:

            filter_result = self.data_collector.process_detection(image=img, detection=detection)
            filter_result_color_dict = {
                CollectionStatus.REJECTED: QColor(255, 0, 0),
                CollectionStatus.REDUNDANT: QColor(255, 192, 0),
                CollectionStatus.ACCEPTED: QColor(0, 255, 0),
            }
            self.image_view.set_draw_detection_color(filter_result_color_dict[filter_result])

            self.data_collection_training_label.setText(
                f"Training samples: {self.data_collector.get_num_training_samples()}"
            )
            self.data_collection_evaluation_label.setText(
                f"Evaluation samples: {self.data_collector.get_num_evaluation_samples()}"
            )

            # object_points = detection.get_object_points()
            odered_image_points = detection.get_ordered_image_points()
            self.image_view.set_detection_ordered_points(odered_image_points)
            self.image_view.set_grid_size_pixels(detection.get_flattened_cell_sizes().mean())

            reprojection_errors = detection.get_reprojection_errors()
            reprojection_errors_norm = np.linalg.norm(reprojection_errors, axis=-1)
            reprojection_error_max = reprojection_errors_norm.max()
            reprojection_error_mean = reprojection_errors_norm.mean()
            reprojection_error_rms = np.sqrt(np.power(reprojection_errors, 2).mean())

            cell_sizes = detection.get_flattened_cell_sizes()
            reprojection_error_max_relative = np.abs(reprojection_errors_norm / cell_sizes).max()
            reprojection_error_mean_relative = np.abs(reprojection_errors_norm / cell_sizes).mean()
            reprojection_error_rms_relative = np.sqrt(
                np.power(reprojection_errors / cell_sizes.reshape(-1, 1), 2).mean()
            )

            pose_rotation, pose_translation = detection.get_pose()
            rough_angles = detection.get_rotation_angles()

            self.raw_detection_label.setText("Detected: True")
            self.raw_linear_error_rms_label.setText(
                f"Linear error rms: {detection.get_linear_error_rms().item():.2f} px"
            )
            self.rough_tilt_label.setText(f"Rough tilt: {detection.get_tilt().item():.2f} degrees")
            self.rough_angles_label.setText(
                f"Rough angles: x={rough_angles[0].item():.2f} y={rough_angles[1].item():.2f} degrees"
            )
            self.rough_position_label.setText(
                f"Rough position: x={pose_translation[0].item():.2f} y={pose_translation[1].item():.2f} z={pose_translation[2].item():.2f}"
            )
            self.skew_label.setText(f"Skew: {detection.get_normalized_skew():.2f}")
            self.relative_area_label.setText(
                f"Relative area: {100.0*detection.get_normalized_size():.2f}"
            )

            self.single_shot_reproj_error_max_label.setText(
                f"Reproj error (max): {reprojection_error_max:.3f} px ({100.0 * reprojection_error_max_relative:.2f}%)"
            )
            self.single_shot_reproj_error_avg_label.setText(
                f"Reproj error (avg): {reprojection_error_mean:.3f} px ({100.0 * reprojection_error_mean_relative:.2f}%)"
            )
            self.single_shot_reproj_error_rms_label.setText(
                f"Reproj error (rms): {reprojection_error_rms:.3f} px ({100.0 * reprojection_error_rms_relative:.2f}%)"
            )

            self.training_occupancy_rate_label.setText(
                f"Training occupancy: {100.0*self.data_collector.get_training_occupancy_rate():.2f}"
            )
            self.evaluation_occupancy_rate_label.setText(
                f"Evaluation occupancy: {100.0*self.data_collector.get_evaluation_occupancy_rate():.2f}"
            )

        # Draw training / evaluation points
        self.image_view.set_draw_training_points(self.draw_training_points_checkbox.isChecked())
        self.image_view.set_draw_evaluation_points(self.draw_evaluation_points_checkbox.isChecked())
        self.image_view.set_draw_training_heatmap(self.draw_training_heatmap_checkbox.isChecked())
        self.image_view.set_draw_evaluation_heatmap(
            self.draw_evaluation_heatmap_checkbox.isChecked()
        )

        if self.draw_training_points_checkbox.isChecked():
            self.image_view.set_training_points(
                self.data_collector.get_flattened_image_training_points()
            )

        if self.draw_evaluation_points_checkbox.isChecked():
            self.image_view.set_evaluation_points(
                self.data_collector.get_flattened_image_evaluation_points()
            )

        if self.draw_training_heatmap_checkbox.isChecked():
            self.image_view.set_training_heatmap(
                self.data_collector.get_training_occupancy_heatmap()
            )

        if self.draw_evaluation_heatmap_checkbox.isChecked():
            self.image_view.set_evaluation_heatmap(
                self.data_collector.get_evaluation_occupancy_heatmap()
            )

        # Set drawing image
        self.image_view.set_image(img)

        # Request a redrawing
        self.image_view.update()
        self.graphics_view.update()
        # print(f"{threading.get_ident()} -> process_detection_results: end")

        self.pending_detection_result = False

        if self.pending_detection_request:
            self.should_process_image.emit()

    def process_data(self):
        # print(f"{threading.get_ident()} -> process_data: start")

        # This should be done in another thread
        img = copy.deepcopy(self.unprocessed_image)
        # self.detector.detect(img)

        self.pending_detection_request = False
        self.pending_detection_result = True

        self.request_image_detection.emit(img)

        # print(f"{threading.get_ident()} -> process_data: end")

    def process_new_data(self):

        if self.paused:
            return

        if self.image_view_mode not in set(
            {ImageViewMode.SOURCE_RECTIFIED, ImageViewMode.SOURCE_UNRECTIFIED}
        ):
            return

        # print("process_new_data")
        with self.lock:
            self.img_id += 1

            if self.produced_image is not None:
                # print("process_new_data: there was a self.produced_image pending !. Overwitting but not emitting!")
                self.unprocessed_image = self.produced_image
            else:
                self.unprocessed_image = self.produced_image
                self.produced_image = None

        if self.pending_detection_result:
            self.pending_detection_request = True
        else:
            self.should_process_image.emit()

    def data_source_external_callback(self, img: np.array):
        """
        Consumer side of the producer/consumer pattern.

        The producer generally has its own thread so synchronization it is neeeded
        Args:
            img (np.array): the produced image coming from any data source
        """
        with self.lock:
            # We depcouple the the data coming from the source with the processing slot to avoid dropping frames in case we cann not process them all
            self.produced_image = img
            self.produced_data_signal.emit()

    def on_parameter_changed(self):

        # print("Some parameter has changed. Should we do anything ?")
        self.should_process_image.emit()

    def sensor_data_callback(self):

        # This method is executed in the UI thread
        with self.lock:

            self.image_view.set_pixmap(self.pixmap_tmp)
            self.image_view.set_pointcloud(self.pointcloud_tmp)

            if not (
                self.use_optimized_intrinsics_checkbox.isChecked()
                and self.use_optimized_intrinsics_checkbox.isEnabled()
            ):
                self.camera_info = self.camera_info_tmp

                self.image_view.set_camera_info(self.camera_info_tmp.k, self.camera_info_tmp.d)
                self.calibrator.set_camera_info(self.camera_info_tmp.k, self.camera_info_tmp.d)

            self.image_view.update()
            self.graphics_view.update()
            pass


def main(args=None):
    app = QApplication(sys.argv)

    rclpy.init(args=args)

    import yaml

    cfg = {}
    with open(sys.argv[1], "r") as stream:
        try:
            cfg = yaml.safe_load(stream)
        except yaml.YAMLError as e:
            print(e)

    try:
        signal.signal(signal.SIGINT, sigint_handler)

        ui = CameraIntrinsicsCalibratorUI(cfg)  # noqa: F841

        sys.exit(app.exec_())
    except (KeyboardInterrupt, SystemExit):
        print("Received sigint. Quiting...")
        rclpy.shutdown()


def sigint_handler(*args):
    QApplication.quit()


if __name__ == "__main__":
    main()
