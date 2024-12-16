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
from collections import defaultdict
import copy
import logging
from optparse import OptionParser
import os
import signal
import sys
import threading
import time
from typing import Dict

from PySide2.QtCore import QThread
from PySide2.QtCore import QTimer
from PySide2.QtCore import Qt
from PySide2.QtCore import Signal
from PySide2.QtGui import QColor
from PySide2.QtWidgets import QApplication
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
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QSlider
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
import cv2
from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.board_detectors.board_detector_factory import make_detector
from intrinsic_camera_calibrator.boards import BoardEnum
from intrinsic_camera_calibrator.calibrators.calibrator import Calibrator
from intrinsic_camera_calibrator.calibrators.calibrator import CalibratorEnum
from intrinsic_camera_calibrator.calibrators.calibrator_factory import make_calibrator
from intrinsic_camera_calibrator.camera_models.camera_model import CameraModel
from intrinsic_camera_calibrator.camera_models.camera_model_factory import make_camera_model
from intrinsic_camera_calibrator.data_collector import CollectionStatus
from intrinsic_camera_calibrator.data_collector import DataCollector
from intrinsic_camera_calibrator.data_sources.data_source import DataSource
from intrinsic_camera_calibrator.parameter import ParameterizedClass
from intrinsic_camera_calibrator.types import ImageViewMode
from intrinsic_camera_calibrator.types import OperationMode
from intrinsic_camera_calibrator.utils import save_intrinsics
from intrinsic_camera_calibrator.utils import set_logger_severity
from intrinsic_camera_calibrator.views.data_collector_view import DataCollectorView
from intrinsic_camera_calibrator.views.image_view import CustomQGraphicsView
from intrinsic_camera_calibrator.views.image_view import ImageView
from intrinsic_camera_calibrator.views.initialization_view import InitializationView
from intrinsic_camera_calibrator.views.parameter_view import ParameterView
import numpy as np
import rclpy
import yaml


class CameraIntrinsicsCalibratorUI(QMainWindow):
    produced_data_signal = Signal()
    consumed_data_signal = Signal()
    should_process_image = Signal()
    request_image_detection = Signal(object, float)

    def __init__(self, cfg):
        super().__init__()
        self.setWindowTitle("Camera intrinsics calibrator")

        self.cfg = defaultdict(dict, cfg)

        # Threading variables
        self.lock = threading.RLock()
        self.produced_image = None
        self.produced_stamp = None
        self.unprocessed_image = None
        self.pending_detection_request = False
        self.pending_detection_result = False

        self.detector_thread = QThread()
        self.detector_thread.start()

        self.calibration_thread = QThread()
        self.calibration_thread.start()

        # Calibration results
        self.estimated_fps = 0
        self.last_processed_stamp = None

        # Camera models to use normally
        self.current_camera_model: CameraModel = None
        self.current_calibrator_type: CalibratorEnum = None
        self.pending_partial_calibration = False

        # Camera model produced via a full calibration
        self.calibrated_camera_model: CameraModel = None

        # Camera model calibrated automatically as we collect data
        self.partial_calibration_distorted_camera_model: CameraModel = None
        self.partial_calibration_undistorted_camera_model: CameraModel = None

        # General Configuration
        self.operation_mode = OperationMode.IDLE
        self.board_type = BoardEnum.CHESS_BOARD
        self.board_parameters: ParameterizedClass = None
        self.detector: BoardDetector = None

        self.image_view_mode = ImageViewMode.SOURCE_UNRECTIFIED
        self.paused = False
        self.last_detection = None
        self.skip_next_img = 3

        self.time_for_debug = time.time()

        self.initialization_view = InitializationView(self, cfg)

    def make_image_view(self):
        self.image_view = ImageView()

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

        data_control_label = QLabel("Data control:")
        self.pause_button = QPushButton("Pause")
        image_view_label = QLabel("Image view type:")
        self.image_view_type_combobox = QComboBox()

        self.training_sample_label = QLabel("Training sample:")
        self.training_sample_slider = QSlider(Qt.Horizontal)
        self.training_sample_slider.setEnabled(False)

        self.evaluation_sample_label = QLabel("Evaluation sample:")
        self.evaluation_sample_slider = QSlider(Qt.Horizontal)
        self.evaluation_sample_slider.setEnabled(False)

        for image_view_type in ImageViewMode:
            self.image_view_type_combobox.addItem(image_view_type.value, image_view_type)

        self.image_view_type_combobox.setEnabled(False)

        def pause_callback():
            if self.paused:
                self.pause_button.setText("Pause")
                self.paused = False
                self.data_source.resume()
            else:
                self.pause_button.setText("Resume")
                self.paused = True
                self.should_process_image.emit()
                self.data_source.pause()

        def on_image_view_type_change(index):
            image_view_type = self.image_view_type_combobox.itemData(index)

            def delayed_change():
                if self.pending_detection_result:
                    QTimer.singleShot(1000, delayed_change)
                    return

                if image_view_type == ImageViewMode.TRAINING_DB_UNRECTIFIED:
                    self.training_sample_slider.setRange(
                        0, self.data_collector.get_num_training_samples() - 1
                    )
                    self.training_sample_slider.setValue(0)
                    self.training_sample_slider.setEnabled(True)
                    self.training_sample_slider.valueChanged.emit(0)

                elif image_view_type == ImageViewMode.EVALUATION_DB_UNRECTIFIED:
                    self.evaluation_sample_slider.setRange(
                        0, self.data_collector.get_num_evaluation_samples() - 1
                    )
                    self.evaluation_sample_slider.setValue(0)
                    self.evaluation_sample_slider.setEnabled(True)
                    self.evaluation_sample_slider.valueChanged.emit(0)
                else:
                    self.training_sample_slider.setEnabled(False)
                    self.evaluation_sample_slider.setEnabled(False)

            if self.pending_detection_result:
                QTimer.singleShot(1000, delayed_change)
            else:
                delayed_change()

        def on_training_sample_changed(index):
            logging.debug(f"on_training_sample_changed={index}")
            self.training_sample_label.setText(f"Training sample: {index}")
            img = self.data_collector.get_training_image(index)
            self.process_db_data(img)

        def on_evaluation_sample_changed(index):
            logging.info(f"on_evaluation_sample_changed={index}")
            self.evaluation_sample_label.setText(f"Evaluation sample: {index}")
            img = self.data_collector.get_evaluation_image(index)
            self.process_db_data(img)

        self.pause_button.clicked.connect(pause_callback)
        self.image_view_type_combobox.currentIndexChanged.connect(on_image_view_type_change)
        self.training_sample_slider.valueChanged.connect(on_training_sample_changed)
        self.evaluation_sample_slider.valueChanged.connect(on_evaluation_sample_changed)

        mode_options_layout = QVBoxLayout()
        mode_options_layout.setAlignment(Qt.AlignTop)
        mode_options_layout.addWidget(data_control_label)
        mode_options_layout.addWidget(self.pause_button)
        mode_options_layout.addWidget(image_view_label)
        mode_options_layout.addWidget(self.image_view_type_combobox)

        mode_options_layout.addWidget(self.training_sample_label)
        mode_options_layout.addWidget(self.training_sample_slider)
        mode_options_layout.addWidget(self.evaluation_sample_label)
        mode_options_layout.addWidget(self.evaluation_sample_slider)

        self.mode_options_group.setLayout(mode_options_layout)

    def make_solver_group(self):
        self.solver_group = QGroupBox("Solver selection")
        self.solver_group.setFlat(True)
        self.calibrator_type_combobox = QComboBox()
        self.calibrator_type_combobox.setEnabled(True)

        def on_calibrator_clicked():
            self.current_calibrator_type = self.calibrator_type_combobox.currentData()
       
        for calibrator_type in CalibratorEnum:
            self.calibrator_type_combobox.addItem(calibrator_type.value["display"], calibrator_type)

        if "calibrator_type" in self.cfg:
            try:
                self.calibrator_type_combobox.setCurrentIndex(
                    CalibratorEnum.from_name(self.cfg["calibrator_type"]).get_id()
                )
            except Exception as e:
                logging.error(f"Invalid calibration_type: {e}")
        else:
            self.calibrator_type_combobox.setCurrentIndex(0)

        self.calibrator_type_combobox.currentIndexChanged.connect(on_calibrator_clicked)
        self.current_calibrator_type = self.calibrator_type_combobox.currentData()
        
        solver_layout = QVBoxLayout()
        solver_layout.setAlignment(Qt.AlignTop)
        solver_layout.addWidget(self.calibrator_type_combobox)
        self.solver_group.setLayout(solver_layout)

    def make_calibration_group(self):
        self.calibration_group = QGroupBox("Calibration control")
        self.calibration_group.setFlat(True)

        self.calibration_parameters_button = QPushButton("Calibration parameters")
        self.calibration_button = QPushButton("Calibrate")
        self.evaluation_button = QPushButton("Evaluate")
        self.save_button = QPushButton("Save")
        self.calibration_status_label = QLabel("Calibration status: idle")
        self.calibration_time_label = QLabel("Calibration time:")
        self.calibration_training_samples_label = QLabel("Training samples:")
        self.calibration_training_pre_rejection_inliers_label = QLabel("\tPre rejection inliers:")
        self.calibration_training_post_rejection_inliers_label = QLabel("\tPost rejection inliers:")
        self.calibration_training_rms_label = QLabel("\trms error (all):")
        self.calibration_training_inlier_rms_label = QLabel("\trms error (inlier):")

        self.calibration_evaluation_samples_label = QLabel("Evaluation samples:")
        self.calibration_evaluation_post_rejection_inliers_label = QLabel(
            "\tPost rejection inliers:"
        )
        self.calibration_evaluation_rms_label = QLabel("\trms error (all):")
        self.calibration_evaluation_inlier_rms_label = QLabel("\trms error (inlier):")

        def on_parameters_view_closed():
            self.calibration_parameters_button.setEnabled(True)
            self.calibrator_type_combobox.setEnabled(True)

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
            self.evaluation_button.setEnabled(False)

            self.calibration_status_label.setText("Calibration status: calibrating")

        def on_evaluation_clicked():
            calibrator_type = self.calibrator_type_combobox.currentData()

            camera_model = (
                self.current_camera_model
                if self.calibrated_camera_model is None
                else self.calibrated_camera_model
            )
            self.calibrator_dict[calibrator_type].evaluation_request.emit(
                self.data_collector.clone_without_images(), camera_model
            )

            self.calibrator_type_combobox.setEnabled(False)
            self.calibration_parameters_button.setEnabled(False)
            self.calibration_button.setEnabled(False)
            self.evaluation_button.setEnabled(False)

            self.calibration_status_label.setText("Calibration status: evaluating")

        self.calibration_parameters_button.clicked.connect(on_parameters_button_clicked)

        self.calibration_button.clicked.connect(on_calibration_clicked)
        self.calibration_button.setEnabled(False)

        self.evaluation_button.clicked.connect(on_evaluation_clicked)
        self.evaluation_button.setEnabled(False)

        self.save_button.clicked.connect(self.on_save_clicked)
        self.save_button.setEnabled(False)

        calibration_layout = QVBoxLayout()
        calibration_layout.setAlignment(Qt.AlignTop)
        calibration_layout.addWidget(self.calibration_parameters_button)
        calibration_layout.addWidget(self.calibration_button)
        calibration_layout.addWidget(self.evaluation_button)
        calibration_layout.addWidget(self.save_button)
        calibration_layout.addWidget(self.calibration_status_label)
        calibration_layout.addWidget(self.calibration_time_label)
        calibration_layout.addWidget(self.calibration_training_samples_label)
        calibration_layout.addWidget(self.calibration_training_pre_rejection_inliers_label)
        calibration_layout.addWidget(self.calibration_training_post_rejection_inliers_label)
        calibration_layout.addWidget(self.calibration_training_rms_label)
        calibration_layout.addWidget(self.calibration_training_inlier_rms_label)

        calibration_layout.addWidget(self.calibration_evaluation_samples_label)
        calibration_layout.addWidget(self.calibration_evaluation_post_rejection_inliers_label)
        calibration_layout.addWidget(self.calibration_evaluation_rms_label)
        calibration_layout.addWidget(self.calibration_evaluation_inlier_rms_label)

        self.calibration_group.setLayout(calibration_layout)

    def make_detector_group(self):
        def detector_parameters_button_callback():
            logging.info("detector_parameters_button_callback")
            self.detector_parameters_view = ParameterView(self.detector)
            self.detector_parameters_view.parameter_changed.connect(self.on_parameter_changed)

        self.detector_options_group = QGroupBox("Detection options")
        self.detector_options_group.setFlat(True)

        self.detector_parameters_button = QPushButton("Detector parameters")

        self.detector_parameters_button.clicked.connect(detector_parameters_button_callback)

        detector_options_layout = QVBoxLayout()
        detector_options_layout.setAlignment(Qt.AlignTop)
        detector_options_layout.addWidget(self.detector_parameters_button)
        self.detector_options_group.setLayout(detector_options_layout)

    def make_detection_group(self):
        self.raw_detection_results_group = QGroupBox("Detection results")
        self.raw_detection_results_group.setFlat(True)

        self.single_shot_detection_results_group = QGroupBox("Single-shot detection results")
        self.single_shot_detection_results_group.setFlat(True)

        self.raw_detection_label = QLabel("Detected:")
        self.raw_linear_error_rows_rms_label = QLabel("Linear error rows (rms):")
        self.raw_linear_error_cols_rms_label = QLabel("Linear error cols (rms):")
        self.aspect_ratio_label = QLabel("Aspect Ratio:")
        self.rough_tilt_label = QLabel("Rough tilt:")
        self.rough_angles_label = QLabel("Rough angles:")
        self.rough_position_label = QLabel("Rough position:")
        self.skew_label = QLabel("Skew:")
        self.relative_area_label = QLabel("Relative area:")

        self.single_shot_reprojection_error_max_label = QLabel("Reprojection error (max):")
        self.single_shot_reprojection_error_avg_label = QLabel("Reprojection error (avg):")
        self.single_shot_reprojection_error_rms_label = QLabel("Reprojection error (rms):")

        raw_detection_results_layout = QVBoxLayout()
        raw_detection_results_layout.setAlignment(Qt.AlignTop)

        single_shot_detection_results_layout = QVBoxLayout()
        single_shot_detection_results_layout.setAlignment(Qt.AlignTop)

        raw_detection_results_layout.addWidget(self.raw_detection_label)
        raw_detection_results_layout.addWidget(self.rough_tilt_label)
        raw_detection_results_layout.addWidget(self.rough_angles_label)
        raw_detection_results_layout.addWidget(self.rough_position_label)
        raw_detection_results_layout.addWidget(self.skew_label)
        raw_detection_results_layout.addWidget(self.relative_area_label)
        raw_detection_results_layout.addWidget(self.raw_linear_error_rows_rms_label)
        raw_detection_results_layout.addWidget(self.raw_linear_error_cols_rms_label)
        raw_detection_results_layout.addWidget(self.aspect_ratio_label)

        single_shot_detection_results_layout.addWidget(
            self.single_shot_reprojection_error_max_label
        )
        single_shot_detection_results_layout.addWidget(
            self.single_shot_reprojection_error_avg_label
        )
        single_shot_detection_results_layout.addWidget(
            self.single_shot_reprojection_error_rms_label
        )

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
            camera_model = (
                self.current_camera_model
                if self.calibrated_camera_model is None
                else self.calibrated_camera_model
            )

            logging.info("view_data_collection_statistics_callback")
            data_collection_statistics_view = DataCollectorView(
                self.data_collector.clone_without_images(), camera_model
            )
            data_collection_statistics_view.plot()

        def data_collection_parameters_closed_callback():
            self.data_collection_parameters_button.setEnabled(True)

        def data_collection_parameters_callback():
            self.data_collection_parameters_button.setEnabled(False)

            logging.debug("data_collection_parameters_callback")
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
        data_collection_layout.setAlignment(Qt.AlignTop)

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

        def draw_linearity_heatmap_callback(value):
            self.image_view.set_draw_linearity_heatmap(value == Qt.Checked)
            self.should_process_image.emit()

        def on_restart_linearity_heatmap_clicked():
            print("restart_lin_heatmap", flush=True)
            self.data_collector.restart_linearity_heatmap()

        self.restart_linearity_heatmap_button = QPushButton("Clear heatmap linearity")
        self.restart_linearity_heatmap_button.clicked.connect(on_restart_linearity_heatmap_clicked)

        self.draw_evaluation_heatmap_checkbox = QCheckBox("Draw evaluation occupancy")
        self.draw_evaluation_heatmap_checkbox.setChecked(False)
        self.draw_evaluation_heatmap_checkbox.stateChanged.connect(draw_evaluation_heatmap_callback)

        self.draw_linearity_heatmap_checkbox = QCheckBox("Draw linearity error")
        self.draw_linearity_heatmap_checkbox.setChecked(False)
        self.draw_linearity_heatmap_checkbox.stateChanged.connect(draw_linearity_heatmap_callback)

        self.draw_indicators_checkbox = QCheckBox("Draw indicators")
        self.draw_indicators_checkbox.setChecked(False)

        rendering_alpha_label = QLabel("Drawings alpha:")

        self.rendering_alpha_spinbox = QDoubleSpinBox()
        self.rendering_alpha_spinbox.setDecimals(2)
        self.rendering_alpha_spinbox.setRange(0.0, 1.0)
        self.rendering_alpha_spinbox.setSingleStep(0.05)
        self.rendering_alpha_spinbox.setValue(1.0)
        self.rendering_alpha_spinbox.valueChanged.connect(lambda: self.should_process_image.emit())

        undistortion_alpha_label = QLabel("Undistortion alpha:")

        self.undistortion_alpha_spinbox = QDoubleSpinBox()
        self.undistortion_alpha_spinbox.setDecimals(2)
        self.undistortion_alpha_spinbox.setRange(0.0, 1.0)
        self.undistortion_alpha_spinbox.setSingleStep(0.05)
        self.undistortion_alpha_spinbox.setValue(0.0)
        self.undistortion_alpha_spinbox.valueChanged.connect(
            lambda: self.should_process_image.emit()
        )
        self.undistortion_alpha_spinbox.valueChanged.connect(on_restart_linearity_heatmap_clicked)
        self.undistortion_alpha_spinbox.setEnabled(False)

        indicators_alpha_label = QLabel("Indicators alpha:")

        self.indicators_alpha_spinbox = QDoubleSpinBox()
        self.indicators_alpha_spinbox.setDecimals(2)
        self.indicators_alpha_spinbox.setRange(0.0, 1.0)
        self.indicators_alpha_spinbox.setSingleStep(0.05)
        self.indicators_alpha_spinbox.setValue(1.0)

        visualization_options_layout = QVBoxLayout()
        visualization_options_layout.setAlignment(Qt.AlignTop)
        visualization_options_layout.addWidget(draw_detection_checkbox)
        visualization_options_layout.addWidget(self.draw_training_points_checkbox)
        visualization_options_layout.addWidget(self.draw_evaluation_points_checkbox)
        visualization_options_layout.addWidget(self.draw_training_heatmap_checkbox)
        visualization_options_layout.addWidget(self.draw_evaluation_heatmap_checkbox)
        visualization_options_layout.addWidget(self.draw_linearity_heatmap_checkbox)
        visualization_options_layout.addWidget(self.draw_indicators_checkbox)
        visualization_options_layout.addWidget(rendering_alpha_label)
        visualization_options_layout.addWidget(self.rendering_alpha_spinbox)
        visualization_options_layout.addWidget(undistortion_alpha_label)
        visualization_options_layout.addWidget(self.undistortion_alpha_spinbox)
        visualization_options_layout.addWidget(indicators_alpha_label)
        visualization_options_layout.addWidget(self.indicators_alpha_spinbox)
        visualization_options_layout.addWidget(self.restart_linearity_heatmap_button)
        self.visualization_options_group.setLayout(visualization_options_layout)

    def start(
        self,
        mode: OperationMode,
        data_source: DataSource,
        board_type: BoardEnum,
        board_parameters: ParameterizedClass,
        initial_intrinsics: CameraModel,
        cfg : dict,
    ):
        self.operation_mode = mode
        self.data_source = data_source
        self.board_type = board_type
        self.board_parameters = board_parameters
        self.current_camera_model = initial_intrinsics

        # if a new parameter file was selected new dictionary is created from those parameters
        self.cfg = defaultdict(dict, cfg)

        self.data_collector = DataCollector(self.cfg["data_collector"])
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
            calibrator.evaluation_results_signal.connect(self.process_evaluation_results)
            calibrator.partial_calibration_results_signal.connect(
                self.process_partial_calibration_result
            )
        
        # Qt logic
        self.should_process_image.connect(self.process_data)
        self.produced_data_signal.connect(self.process_new_data)
        self.consumed_data_signal.connect(self.on_consumed)

        self.central_widget = QWidget(self)
        self.setCentralWidget(self.central_widget)
        self.layout = QHBoxLayout(self.central_widget)

        # Image View
        self.make_image_view()

        # Menu Widgets
        self.left_menu_widget = QWidget(self.central_widget)
        self.left_menu_widget.setFixedWidth(300)
        self.left_menu_layout = QVBoxLayout(self.left_menu_widget)
        self.left_menu_layout.setAlignment(Qt.AlignTop)

        self.right_menu_widget = QWidget(self.central_widget)
        self.right_menu_widget.setFixedWidth(300)
        self.right_menu_layout = QVBoxLayout(self.right_menu_widget)
        self.right_menu_layout.setAlignment(Qt.AlignTop)
        self.setEnabled(False)

        # Creating the UI elements after selecting CALIBRATION or EVALUATION
        # Mode group
        self.make_mode_group()

        # Create solver selector
        self.make_solver_group()

        # Calibration group
        if self.operation_mode == OperationMode.CALIBRATION:
            self.make_calibration_group()

        # Detector group
        self.make_detector_group()

        # Detections group
        self.make_detection_group()

        # Data collection group
        if self.operation_mode == OperationMode.CALIBRATION:
            self.make_data_collection_group()

        # Visualization group
        self.make_visualization_group()

        # self.menu_layout.addWidget(label)
        self.left_menu_layout.addWidget(self.solver_group)
        if self.operation_mode == OperationMode.CALIBRATION:
            self.left_menu_layout.addWidget(self.calibration_group)
        self.left_menu_layout.addWidget(self.detector_options_group)
        self.left_menu_layout.addWidget(self.raw_detection_results_group)
        self.left_menu_layout.addWidget(self.single_shot_detection_results_group)

        self.right_menu_layout.addWidget(self.mode_options_group)
        if self.operation_mode == OperationMode.CALIBRATION:
            self.right_menu_layout.addWidget(self.data_collection_group)
        self.right_menu_layout.addWidget(self.visualization_options_group)

        self.layout.addWidget(self.graphics_view)

        self.layout.addWidget(self.left_menu_widget)
        self.layout.addWidget(self.right_menu_widget)
        self.show()
        self.setEnabled(True)

        if self.operation_mode == OperationMode.CALIBRATION:
            self.setWindowTitle(
                f"Camera intrinsics calibrator ({self.data_source.get_camera_name()})"
            )
        if self.operation_mode == OperationMode.EVALUATION:
            self.setWindowTitle(
                f"Camera intrinsics Evaluation Mode ({self.data_source.get_camera_name()})"
            )

        logging.info("Init")
        logging.info(f"\tmode : {mode}")
        logging.info(f"\tdata_source : {data_source}")
        logging.info(f"\tboard_type : {board_type}")

        detector_cfg = self.cfg[self.board_type.value["name"] + "_detector"]

        self.detector = make_detector(
            board_type=self.board_type,
            lock=self.lock,
            board_parameters=self.board_parameters,
            cfg=detector_cfg,
        )

        if self.operation_mode == OperationMode.EVALUATION:
            #  Initial state of the elements on evaluation mode
            self.calibrated_camera_model = self.current_camera_model
            self.image_view_type_combobox.setEnabled(True)
            self.undistortion_alpha_spinbox.setEnabled(True)
            self.draw_evaluation_heatmap_checkbox.setEnabled(False)
            self.draw_evaluation_points_checkbox.setEnabled(False)
            self.draw_training_points_checkbox.setEnabled(False)
            self.draw_training_heatmap_checkbox.setEnabled(False)
            self.training_sample_slider.setEnabled(False)
            self.evaluation_sample_slider.setEnabled(False)
            self.image_view_type_combobox.clear()
            # Order of of how items are added to the combobox matters,
            # default index is 0, so rectified image is added first to be default view
            self.image_view_type_combobox.addItem(
                ImageViewMode.SOURCE_RECTIFIED.value, ImageViewMode.SOURCE_RECTIFIED
            )
            self.image_view_type_combobox.addItem(
                ImageViewMode.SOURCE_UNRECTIFIED.value, ImageViewMode.SOURCE_UNRECTIFIED
            )

        self.detector.moveToThread(self.detector_thread)
        self.detector.detection_results_signal.connect(self.process_detection_results)
        self.request_image_detection.connect(self.detector.detect)

    def process_calibration_results(
        self,
        calibrated_model: CameraModel,
        dt: float,
        num_training_detections: int,
        num_training_pre_rejection_inliers: int,
        num_training_post_rejection_inliers: int,
        training_rms_error: float,
        training_inlier_rms_error: float,
        num_evaluation_detections: int,
        num_evaluation_post_rejection_inliers: int,
        evaluation_rms_error: float,
        evaluation_inlier_rms_error: float,
    ):
        self.image_view_type_combobox.setEnabled(True)
        self.undistortion_alpha_spinbox.setEnabled(True)
        self.current_camera_model = calibrated_model
        self.calibrated_camera_model = calibrated_model

        self.calibration_status_label.setText("Calibration status: idle")
        self.calibration_time_label.setText(f"Calibration time: {dt:.2f}s")  # noqa E231
        self.calibration_training_samples_label.setText(
            f"Training samples: {num_training_detections}"
        )
        self.calibration_training_pre_rejection_inliers_label.setText(
            f"\tPre rejection inliers: {num_training_pre_rejection_inliers}"
        )
        self.calibration_training_post_rejection_inliers_label.setText(
            f"\tPost rejection inliers: {num_training_post_rejection_inliers}"
        )

        self.calibration_training_rms_label.setText(
            f"\trms error (all): {training_rms_error:.3f}"  # noqa E231
        )
        self.calibration_training_inlier_rms_label.setText(
            f"\trms error (inliers): {training_inlier_rms_error:.3f}"  # noqa E231
        )

        self.calibration_evaluation_samples_label.setText(
            f"Evaluation samples: {num_evaluation_detections}"
        )
        self.calibration_evaluation_post_rejection_inliers_label.setText(
            f"\tPost rejection inliers: {num_evaluation_post_rejection_inliers}"
        )

        self.calibration_evaluation_rms_label.setText(
            f"\trms error (all): {evaluation_rms_error:.3f}"  # noqa E231
        )
        self.calibration_evaluation_inlier_rms_label.setText(
            f"\trms error (inliers): {evaluation_inlier_rms_error:.3f}"  # noqa E231
        )

        self.calibrator_type_combobox.setEnabled(True)
        self.calibration_parameters_button.setEnabled(True)
        self.calibration_button.setEnabled(True)
        self.evaluation_button.setEnabled(True)
        self.save_button.setEnabled(True)

    def process_evaluation_results(
        self,
        dt: float,
        num_training_detections: int,
        num_training_post_rejection_inliers: int,
        training_rms_error: float,
        training_inlier_rms_error: float,
        num_evaluation_detections: int,
        num_evaluation_post_rejection_inliers: int,
        evaluation_rms_error: float,
        evaluation_inlier_rms_error: float,
    ):
        self.image_view_type_combobox.setEnabled(True)
        self.undistortion_alpha_spinbox.setEnabled(True)

        self.calibration_status_label.setText("Calibration status: idle")
        self.calibration_time_label.setText(f"Calibration time: {dt:.2f}s")  # noqa E231
        self.calibration_training_samples_label.setText(
            f"Training samples: {num_training_detections}"
        )
        self.calibration_training_pre_rejection_inliers_label.setText("\tPre rejection inliers:")
        self.calibration_training_post_rejection_inliers_label.setText(
            f"\tPost rejection inliers: {num_training_post_rejection_inliers}"
        )

        self.calibration_training_rms_label.setText(
            f"\trms error (all): {training_rms_error:.3f}"  # noqa E231
        )
        self.calibration_training_inlier_rms_label.setText(
            f"\trms error (inliers): {training_inlier_rms_error:.3f}"  # noqa E231
        )

        self.calibration_evaluation_samples_label.setText(
            f"Evaluation samples: {num_evaluation_detections}"
        )
        self.calibration_evaluation_post_rejection_inliers_label.setText(
            f"\tPost rejection inliers: {num_evaluation_post_rejection_inliers}"
        )

        self.calibration_evaluation_rms_label.setText(
            f"\trms error (all): {evaluation_rms_error:.3f}"  # noqa E231
        )
        self.calibration_evaluation_inlier_rms_label.setText(
            f"\trms error (inliers): {evaluation_inlier_rms_error:.3f}"  # noqa E231
        )

        self.calibrator_type_combobox.setEnabled(True)
        self.calibration_parameters_button.setEnabled(True)
        self.calibration_button.setEnabled(self.operation_mode == OperationMode.CALIBRATION)
        self.evaluation_button.setEnabled(True)
        self.save_button.setEnabled(True)

    def on_consumed(self):
        self.data_source.consumed()

    def save_parameters(self, filename):
        data_coll_params = self.data_collector.get_parameters_values()
        board_params = self.board_parameters.get_parameters_values()
        detector_params = self.detector.get_parameters_values()
        calibrator_type = self.calibrator_type_combobox.currentData()
        print(calibrator_type.value["name"], flush=True)
        calib_params = self.calibrator_dict[calibrator_type].get_parameters_values()
        with open(filename, "w") as file:
            yaml.dump({"board_parameters": board_params}, file, default_flow_style=False)
            yaml.dump({"board_type" : self.board_type.value["name"]}, file, default_flow_style=False)
            yaml.dump(
                {"calibrator_type": calibrator_type.value["name"]}, file, default_flow_style=False
            )
            yaml.dump({"calibration_parameters": calib_params}, file, default_flow_style=False)
            yaml.dump({"data_collector": data_coll_params}, file, default_flow_style=False)
            yaml.dump({"detector_params": detector_params}, file, default_flow_style=False)

    def on_save_clicked(self):
        output_folder = QFileDialog.getExistingDirectory(
            None,
            "Select directory to save the calibration result",
            ".",
            QFileDialog.ShowDirsOnly | QFileDialog.DontResolveSymlinks,
        )

        if output_folder is None or output_folder == "":
            return

        logging.info(f"Saving calibration results to {output_folder}")

        save_intrinsics(
            self.calibrated_camera_model,
            self.undistortion_alpha_spinbox.value(),
            self.data_source.get_camera_name(),
            os.path.join(output_folder, f"{self.data_source.get_camera_name()}_info.yaml"),
        )

        self.save_parameters(os.path.join(output_folder, "parameters.yaml"))

        training_folder = os.path.join(output_folder, "training_images")
        evaluation_folder = os.path.join(output_folder, "evaluation_images")

        if not os.path.exists(training_folder):
            os.mkdir(training_folder)
        if not os.path.exists(evaluation_folder):
            os.mkdir(evaluation_folder)

        for index, image in enumerate(self.data_collector.get_training_images()):
            cv2.imwrite(os.path.join(training_folder, f"{index:04d}.jpg"), image)  # noqa E231
            np.savetxt(
                os.path.join(training_folder, f"{index:04d}_training_img_points.txt"),
                self.data_collector.get_training_detection(index).get_flattened_image_points(),
            )
            np.savetxt(
                os.path.join(training_folder, f"{index:04d}_training_obj_points.txt"),
                self.data_collector.get_training_detection(index).get_flattened_object_points(),
            )

        for index, image in enumerate(self.data_collector.get_evaluation_images()):
            cv2.imwrite(os.path.join(evaluation_folder, f"{index:04d}.jpg"), image)  # noqa E231
            np.savetxt(
                os.path.join(evaluation_folder, f"{index:04d}_eval_img_points.txt"),
                self.data_collector.get_evaluation_detection(index).get_flattened_image_points(),
            )
            np.savetxt(
                os.path.join(evaluation_folder, f"{index:04d}_eval_obj_points.txt"),
                self.data_collector.get_evaluation_detection(index).get_flattened_object_points(),
            )

    def process_detection_results(self, img: np.array, detection: BoardDetection, img_stamp: float):
        """Process the results from an object detection."""
        # Signal that the detector is free
        print("elapsedtime_Debug: ", time.time()- self.time_for_debug, flush=True)
        self.consumed_data_signal.emit()
        self.time_for_debug = time.time()

        if img is None:
            self.pending_detection_result = False

            if self.pending_detection_request:
                self.should_process_image.emit()

            return

        # Set general options
        self.image_view.set_rendering_alpha(self.rendering_alpha_spinbox.value())

        # Set detection drawings
        if detection is None:
            self.image_view.set_detection_ordered_points(None)

            self.raw_detection_label.setText("Detected: False")
            self.raw_linear_error_rows_rms_label.setText("Linear error rows rms:")
            self.raw_linear_error_cols_rms_label.setText("Linear error cols rms:")
            self.aspect_ratio_label.setText("Aspect Ratio:")
            self.rough_tilt_label.setText("Rough tilt:")
            self.rough_angles_label.setText("Rough angles:")
            self.rough_position_label.setText("Rough position:")
            self.skew_label.setText("Skew:")
            self.relative_area_label.setText("Relative area:")

            self.single_shot_reprojection_error_max_label.setText("Reprojection error (max):")
            self.single_shot_reprojection_error_avg_label.setText("Reprojection error (avg):")
            self.single_shot_reprojection_error_rms_label.setText("Reprojection error (rms):")
            board_speed = None
            self.image_view.set_draw_indicators(
                board_speed,
                self.data_collector.max_allowed_pixel_speed.value,
                self.data_collector.get_skew_percentage(),
                self.data_collector.get_size_percentage(),
                0,
                0,  # rows cols linear error
                0,
                0,  # rows cols percentage linear error
                0.0,  # aspect ratio
                0,
                0,
                self.indicators_alpha_spinbox.value(),
                False,
            )
            self.skip_next_img = 3 # skips the next images if there are no detections

        else:
            camera_model_cfg, camera_model_type = self.calibrator_dict[
                self.current_calibrator_type
            ].get_model_info()
            camera_model = make_camera_model(camera_model_type)
            camera_model.update_config(**camera_model_cfg)

            if self.image_view_type_combobox.currentData() == ImageViewMode.SOURCE_UNRECTIFIED:
                dat_coll_start = time.time()
                filter_result = self.data_collector.process_detection(
                    image=img,
                    detection=detection,
                    camera_model=camera_model,
                    mode=self.operation_mode,
                )
                # filter_result = CollectionStatus.REJECTED
                print("dat_coll : ", time.time()-dat_coll_start, flush=True)
            else:
                filter_result = CollectionStatus.NOT_EVALUATED

            if (
                self.operation_mode == OperationMode.EVALUATION
                and self.image_view_type_combobox.currentData() == ImageViewMode.SOURCE_RECTIFIED
            ):
                self.data_collector.process_detection_eval_mode(
                    image=img,
                    detection=detection,
                    camera_model=camera_model,
                    mode=self.operation_mode,
                )

            # For each new sample that is accepted we try to update the current (partial) calibration
            if (
                filter_result == CollectionStatus.ACCEPTED
                and self.operation_mode == OperationMode.CALIBRATION
            ):
                self.update_current_camera_model()

            filter_result_color_dict = {
                CollectionStatus.NOT_EVALUATED: QColor(255, 255, 255),
                CollectionStatus.REJECTED: QColor(255, 0, 0),
                CollectionStatus.REDUNDANT: QColor(255, 192, 0),
                CollectionStatus.ACCEPTED: QColor(0, 255, 0),
            }
            self.image_view.set_draw_detection_color(filter_result_color_dict[filter_result])

            if self.operation_mode == OperationMode.CALIBRATION:
                self.data_collection_training_label.setText(
                    f"Training samples: {self.data_collector.get_num_training_samples()}"
                )
                self.data_collection_evaluation_label.setText(
                    f"Evaluation samples: {self.data_collector.get_num_evaluation_samples()}"
                )

            # object_points = detection.get_object_points()
            ordered_image_points = detection.get_ordered_image_points()
            self.image_view.set_detection_ordered_points(ordered_image_points)
            self.image_view.set_grid_size_pixels(detection.get_flattened_cell_sizes().mean())
            repr_err_time = time.time()
            reprojection_errors = detection.get_reprojection_errors(camera_model)
            print("repr_err_time: ", time.time() - repr_err_time, flush=True)
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

            pose_rotation, pose_translation = detection.get_pose(camera_model)
            pose_translation = pose_translation.flatten()
            rough_angles = detection.get_rotation_angles(camera_model)
            self.raw_detection_label.setText("Detected: True")
            err_rms_rows, err_rms_cols, pct_err_rows, pct_err_cols = (
                detection.get_linear_error_rms()
            )
            self.raw_linear_error_rows_rms_label.setText(
                f"Linear error rows rms:  {err_rms_rows:.2f} px"  # noqa E231
            )
            self.raw_linear_error_cols_rms_label.setText(
                f"Linear error cols rms:  {err_rms_cols:.2f} px"  # noqa E231
            )
            self.aspect_ratio_label.setText(
               f"Aspect Ratio:  {detection.get_aspect_ratio_pattern(camera_model):.2f} px"  # noqa E231
            )
            self.rough_tilt_label.setText(
                f"Rough tilt: {detection.get_tilt(camera_model):.2f} degrees"  # noqa E231
            )
            self.rough_angles_label.setText(
                f"Rough angles: x={rough_angles[0]:.2f} y={rough_angles[1]:.2f} degrees"  # noqa E231
            )
            self.rough_position_label.setText(
                f"Rough position: x={pose_translation[0]:.2f} y={pose_translation[1]:.2f} z={pose_translation[2]:.2f}"  # noqa E231
            )
            self.skew_label.setText(f"Skew: {detection.get_normalized_skew():.2f}")  # noqa E231
            self.relative_area_label.setText(
                f"Relative area: {100.0*detection.get_normalized_size():.2f}"  # noqa E231
            )

            self.single_shot_reprojection_error_max_label.setText(
                f"Reprojection error (max): {reprojection_error_max:.3f} px ({100.0 * reprojection_error_max_relative:.2f}%)"  # noqa E231
            )
            self.single_shot_reprojection_error_avg_label.setText(
                f"Reprojection error (avg): {reprojection_error_mean:.3f} px ({100.0 * reprojection_error_mean_relative:.2f}%)"  # noqa E231
            )
            self.single_shot_reprojection_error_rms_label.setText(
                f"Reprojection error (rms): {reprojection_error_rms:.3f} px ({100.0 * reprojection_error_rms_relative:.2f}%)"  # noqa E231
            )

            if self.operation_mode == OperationMode.CALIBRATION:
                self.training_occupancy_rate_label.setText(
                    f"Training occupancy: {100.0*self.data_collector.get_training_occupancy_rate():.2f}"  # noqa E231
                )
                self.evaluation_occupancy_rate_label.setText(
                    f"Evaluation occupancy: {100.0*self.data_collector.get_evaluation_occupancy_rate():.2f}"  # noqa E231
                )

            board_speed = (
                0 if self.last_detection is None else detection.get_speed(self.last_detection)
            )
            self.last_detection = detection
            pan, tilt = rough_angles
            self.image_view.set_draw_indicators(
                board_speed,
                self.data_collector.max_allowed_pixel_speed.value,
                self.data_collector.get_skew_percentage(),
                self.data_collector.get_size_percentage(),
                err_rms_rows,
                err_rms_cols,
                pct_err_rows,
                pct_err_cols,
                detection.get_aspect_ratio_pattern(camera_model),
                pan,
                tilt,
                self.indicators_alpha_spinbox.value(),
                self.draw_indicators_checkbox.isChecked(),
            )

        # Draw training / evaluation points
        self.image_view.set_draw_training_points(self.draw_training_points_checkbox.isChecked())
        self.image_view.set_draw_evaluation_points(self.draw_evaluation_points_checkbox.isChecked())
        self.image_view.set_draw_training_heatmap(self.draw_training_heatmap_checkbox.isChecked())
        self.image_view.set_draw_evaluation_heatmap(
            self.draw_evaluation_heatmap_checkbox.isChecked()
        )

        self.image_view.set_draw_linearity_heatmap(self.draw_linearity_heatmap_checkbox.isChecked())

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

        if self.draw_linearity_heatmap_checkbox.isChecked():
            self.image_view.set_linearity_heatmap(self.data_collector.get_linearity_heatmap())

        if self.operation_mode == OperationMode.CALIBRATION:
            if (
                self.data_collector.get_num_training_samples() > 0
                and not self.calibration_button.isEnabled()
            ):
                self.calibration_button.setEnabled(True)

            if (
                self.data_collector.get_num_evaluation_samples() > 0
                and not self.evaluation_button.isEnabled()
            ):
                self.evaluation_button.setEnabled(True)

        # Set drawing image
        self.image_view.set_image(img)

        # Request a redrawing
        current_time = time.time()
        detection_delay = time.time() - img_stamp
        current_fps = (
            0
            if self.last_processed_stamp is None
            else 1.0 / max(1e-5, (current_time - self.last_processed_stamp))
        )
        self.estimated_fps = 0.9 * self.estimated_fps + 0.1 * current_fps
        self.last_processed_stamp = current_time
        detection_time = current_time - self.detection_request_time

        if self.operation_mode == OperationMode.CALIBRATION:
            self.setWindowTitle(f"Camera intrinsics calibrator ({self.data_source.get_camera_name()}). Data delay={detection_delay: .2f} Detection time={detection_time: .2f} fps={self.estimated_fps: .2f} Data time={img_stamp: .2f}"
            )
        if self.operation_mode == OperationMode.EVALUATION:
            self.setWindowTitle(f"Camera intrinsics Evaluation Mode ({self.data_source.get_camera_name()}). Data delay={detection_delay: .2f} Detection time={detection_time: .2f} fps={self.estimated_fps: .2f} Data time={img_stamp: .2f}"
            )

        self.image_view.update()
        self.graphics_view.update()

        self.pending_detection_result = False

        # If there was a pending detection, we process it now
        if self.pending_detection_request:
            self.should_process_image.emit()

    def update_current_camera_model(self):
        """Send a request to update the current camera model."""
        if self.pending_partial_calibration:
            return

        calibrator_type = self.calibrator_type_combobox.currentData()
        self.calibrator_dict[calibrator_type].partial_calibration_request.emit(
            self.data_collector.clone_without_images(), self.current_camera_model
        )

        self.pending_partial_calibration = True

    def process_partial_calibration_result(self, camera_model):
        self.current_camera_model = camera_model
        self.pending_partial_calibration = False

    def process_data(self):
        """Request the detector to process the image (the detector itself runs in another thread). Depending on the ImageViewMode selected, the image is also rectified."""
        stamp = self.unprocessed_stamp

        if self.image_view_type_combobox.currentData() in {
            ImageViewMode.SOURCE_UNRECTIFIED,
            ImageViewMode.TRAINING_DB_UNRECTIFIED,
            ImageViewMode.EVALUATION_DB_UNRECTIFIED,
        }:
            img = copy.deepcopy(self.unprocessed_image)
        elif self.image_view_type_combobox.currentData() == ImageViewMode.SOURCE_RECTIFIED:
            assert self.calibrated_camera_model is not None
            img = self.calibrated_camera_model.rectify(
                self.unprocessed_image, self.undistortion_alpha_spinbox.value()
            )
        else:
            raise NotImplementedError

        self.pending_detection_request = False
        self.pending_detection_result = True
        self.detection_request_time = time.time()

        self.request_image_detection.emit(img, stamp)

    def process_db_data(self, img):
        assert self.image_view_type_combobox.currentData() in set(
            {ImageViewMode.TRAINING_DB_UNRECTIFIED, ImageViewMode.EVALUATION_DB_UNRECTIFIED}
        )

        with self.lock:
            self.unprocessed_image = img

        if self.pending_detection_result:
            self.pending_detection_request = True
        else:
            self.should_process_image.emit()

    def process_new_data(self):
        """Attempt to request the detector to process an image. However, if it there is an image being processed, does not enqueue them indefinitely. Instead, only leave the last one."""
        # if was not found the pattern skip some frames
        if self.skip_next_img > 0:
            print("skipping frame", flush=True)
            self.skip_next_img -= 1
            self.consumed_data_signal.emit()
            return
        if self.paused:
            return

        if self.image_view_type_combobox.currentData() not in set(
            {ImageViewMode.SOURCE_RECTIFIED, ImageViewMode.SOURCE_UNRECTIFIED}
        ):
            return

        with self.lock:
            if self.produced_image is not None:
                self.unprocessed_image = self.produced_image
                self.unprocessed_stamp = self.produced_stamp
            else:
                self.unprocessed_image = self.produced_image
                self.unprocessed_stamp = self.produced_stamp
                self.produced_image = None
                self.produced_stamp = None

        if self.pending_detection_result:
            self.pending_detection_request = True
        else:
            self.should_process_image.emit()

    def data_source_external_callback(self, img: np.array, stamp: float):
        """
        Consumer side of the producer/consumer pattern.

        The producer generally has its own thread so synchronization it is needed
        Args:
            img (np.array): the produced image coming from any data source.
            stamp (float): the produced image's timestamp as a float.
        """
        # We decouple the the data coming from the source with the processing slot to avoid dropping frames in case we can not process them all
        with self.lock:
            self.produced_image = img
            self.produced_stamp = stamp
            # Using a signal from another thread results in the slot being executed in the class Qt thread
            self.produced_data_signal.emit()

    def on_parameter_changed(self):
        self.should_process_image.emit()


def main(args=None):
    set_logger_severity()
    parser = OptionParser()
    parser.add_option("-c", "--config-file", type="string", help="calibration file path")

    (options, args) = parser.parse_args(rclpy.utilities.remove_ros_args())
    if len(args) != 1:
        parser.error(f"incorrect number of arguments: {len(args)}")

    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = ""
    app = QApplication(sys.argv)

    cfg = {}
    try:
        with open(options.config_file, "r") as stream:
            cfg = yaml.safe_load(stream)
    except Exception as e:
        logging.error(f"Could not load the parameters from the YAML file ({e})")

    try:
        signal.signal(signal.SIGINT, sigint_handler)

        ui = CameraIntrinsicsCalibratorUI(cfg)  # noqa: F841
        sys.exit(app.exec_())
    except (KeyboardInterrupt, SystemExit):
        logging.info("Received sigint. Quitting...")
        rclpy.shutdown()


def sigint_handler(*args):
    QApplication.quit()


if __name__ == "__main__":
    main()
