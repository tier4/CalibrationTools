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

from enum import Enum
import multiprocessing as mp
import threading
import time
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

from PySide2.QtCore import QObject
from PySide2.QtCore import Signal
from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.calibrators.utils import (
    plot_calibration_data_statistics as _plot_calibration_data_statistics,
)
from intrinsic_camera_calibrator.calibrators.utils import (
    plot_calibration_results_statistics as _plot_calibration_results_statistics,
)
from intrinsic_camera_calibrator.calibrators.utils import add_detection
from intrinsic_camera_calibrator.calibrators.utils import get_entropy
from intrinsic_camera_calibrator.camera_model import CameraModel
from intrinsic_camera_calibrator.data_collector import DataCollector
from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.parameter import ParameterizedClass
import numpy as np


class CalibratorEnum(Enum):
    OPENCV = {"name": "opencv", "display": "OpenCV"}
    CERES = {"name": "ceres", "display": "Ceres"}

    def from_name(name: str) -> "CalibratorEnum":
        for calibrator_type in CalibratorEnum:
            if name == calibrator_type.value["name"]:
                return calibrator_type

        raise ValueError

    def from_index(i: int):
        return list(CalibratorEnum)[i]

    def get_id(self) -> int:
        for index, calibrator_type in enumerate(CalibratorEnum):
            if calibrator_type == self:
                return index

        raise ValueError


class Calibrator(ParameterizedClass, QObject):
    """Base class of camera intrinsic calibrator. Most of the logic should be implemented here abd the subclasses only implement the core method."""

    calibration_request = Signal(object)
    calibration_results_signal = Signal(
        object, float, int, int, int, float, float, int, int, float, float
    )

    # cSpell:enableCompoundWords
    evaluation_request = Signal(object, object)
    evaluation_results_signal = Signal(float, int, int, float, float, int, int, float, float)

    partial_calibration_request = Signal(object, object)
    partial_calibration_results_signal = Signal(object)

    def __init__(self, lock: threading.RLock, cfg: Optional[Dict] = {}):
        ParameterizedClass.__init__(self, lock)
        QObject.__init__(self, None)

        self.use_ransac_pre_rejection = Parameter(bool, value=True, min_value=False, max_value=True)
        self.pre_rejection_iterations = Parameter(int, value=100, min_value=1, max_value=100)
        self.pre_rejection_min_hypotheses = Parameter(int, value=6, min_value=1, max_value=20)
        self.pre_rejection_max_rms_error = Parameter(
            float, value=0.35, min_value=0.001, max_value=10.0
        )

        self.max_calibration_samples = Parameter(int, value=80, min_value=10, max_value=1000)
        self.max_fast_calibration_samples = Parameter(int, value=20, min_value=1, max_value=100)

        self.use_entropy_maximization_subsampling = Parameter(
            bool, value=True, min_value=False, max_value=True
        )
        self.subsampling_pixel_cells = Parameter(int, value=16, min_value=2, max_value=100)
        self.subsampling_tilt_resolution = Parameter(float, value=15, min_value=5, max_value=45)
        self.subsampling_max_tilt_deg = Parameter(float, value=45, min_value=30, max_value=90)

        self.use_post_rejection = Parameter(bool, value=True, min_value=False, max_value=True)
        self.post_rejection_max_rms_error = Parameter(
            float, value=0.25, min_value=0.001, max_value=10.0
        )

        self.plot_calibration_data_statistics = Parameter(
            bool, value=True, min_value=False, max_value=True
        )
        self.plot_calibration_results_statistics = Parameter(
            bool, value=True, min_value=False, max_value=True
        )

        self.viz_pixel_cells = Parameter(int, value=16, min_value=2, max_value=100)
        self.viz_tilt_resolution = Parameter(float, value=15, min_value=5, max_value=45)
        self.viz_max_tilt_deg = Parameter(float, value=45, min_value=30, max_value=90)
        self.viz_z_cells = Parameter(int, value=12, min_value=2, max_value=100)

        self.set_parameters(**cfg)

        self.calibration_request.connect(self._calibrate)
        self.evaluation_request.connect(self._evaluate)
        self.partial_calibration_request.connect(self._calibrate_fast)

    def _calibrate(self, data_collector: DataCollector):
        """
        General calibration routine.

        1) rejects outlier via ransac
        2) subsample the outliers if necessary
        3) calibrate with the subsamples inliers
        4) calibrates again with only the inliers.
        """
        with self.lock:
            use_ransac_pre_rejection = self.use_ransac_pre_rejection.value
            max_calibration_samples = self.max_calibration_samples.value
            use_entropy_maximization_subsampling = self.use_entropy_maximization_subsampling.value
            use_post_rejection = self.use_post_rejection.value
            plot_calibration_data_statistics = self.plot_calibration_data_statistics.value
            plot_calibration_results_statistics = self.plot_calibration_results_statistics.value

        t0 = time.time()

        raw_training_detections = data_collector.get_training_detections()
        raw_evaluation_detections = data_collector.get_evaluation_detections()

        num_training_detections = len(raw_training_detections)
        num_evaluation_detections = len(raw_evaluation_detections)

        calibrated_model = None

        if use_ransac_pre_rejection:
            calibrated_model, pre_rejection_inliers = self._pre_rejection_filter_impl(
                raw_training_detections
            )
        else:
            pre_rejection_inliers = raw_training_detections

        num_training_pre_rejection_inliers = len(pre_rejection_inliers)

        if num_training_pre_rejection_inliers > max_calibration_samples:
            if use_entropy_maximization_subsampling:
                calibration_training_detections = self._entropy_maximization_subsampling_impl(
                    pre_rejection_inliers, calibrated_model
                )
            else:
                calibration_training_detections = self._random_subsampling_impl(
                    pre_rejection_inliers, max_calibration_samples
                )
        else:
            calibration_training_detections = pre_rejection_inliers

        calibrated_model = self._calibration_impl(calibration_training_detections)

        if use_post_rejection:
            training_post_rejection_inliers = self._post_rejection_impl(
                calibration_training_detections, calibrated_model
            )

            evaluation_post_rejection_inliers = self._post_rejection_impl(
                raw_evaluation_detections, calibrated_model
            )

            calibrated_model = (
                self._calibration_impl(training_post_rejection_inliers)
                if len(training_post_rejection_inliers)
                else calibrated_model
            )
        else:
            training_post_rejection_inliers = calibration_training_detections
            evaluation_post_rejection_inliers = raw_evaluation_detections

        num_training_post_rejection_inliers = len(training_post_rejection_inliers)
        num_evaluation_post_rejection_inliers = len(evaluation_post_rejection_inliers)

        tf = time.time()
        dt = tf - t0

        training_reprojection_errors = [
            calibrated_model.get_reprojection_errors(detection)
            for detection in raw_training_detections
        ]
        training_inlier_reprojection_errors = [
            calibrated_model.get_reprojection_errors(detection)
            for detection in training_post_rejection_inliers
        ]

        evaluation_reprojection_errors = [
            calibrated_model.get_reprojection_errors(detection)
            for detection in raw_evaluation_detections
        ]
        evaluation_inlier_reprojection_errors = [
            calibrated_model.get_reprojection_errors(detection)
            for detection in evaluation_post_rejection_inliers
        ]

        training_rms_error = np.sqrt(
            np.power(np.concatenate(training_reprojection_errors, axis=0), 2).mean()
        )
        training_inlier_rms_error = (
            np.sqrt(np.power(np.concatenate(training_inlier_reprojection_errors, axis=0), 2).mean())
            if len(training_inlier_reprojection_errors) > 0
            else np.inf
        )

        evaluation_rms_error = (
            np.sqrt(np.power(np.concatenate(evaluation_reprojection_errors, axis=0), 2).mean())
            if len(evaluation_reprojection_errors) > 0
            else np.inf
        )
        evaluation_inlier_rms_error = (
            np.sqrt(
                np.power(np.concatenate(evaluation_inlier_reprojection_errors, axis=0), 2).mean()
            )
            if len(evaluation_inlier_reprojection_errors) > 0
            else np.inf
        )

        if plot_calibration_data_statistics:
            self._plot_calibration_data_statistics_impl(
                calibrated_model,
                raw_training_detections,
                pre_rejection_inliers,
                calibration_training_detections,
                training_post_rejection_inliers,
                raw_evaluation_detections,
            )

        if plot_calibration_results_statistics:
            self._plot_calibration_results_statistics(
                calibrated_model,
                raw_training_detections,
                training_post_rejection_inliers,
                raw_evaluation_detections,
            )

        self.calibration_results_signal.emit(
            calibrated_model,
            dt,
            num_training_detections,
            num_training_pre_rejection_inliers,
            num_training_post_rejection_inliers,
            training_rms_error,
            training_inlier_rms_error,
            num_evaluation_detections,
            num_evaluation_post_rejection_inliers,
            evaluation_rms_error,
            evaluation_inlier_rms_error,
        )

    def _evaluate(self, data_collector: DataCollector, evaluation_model: CameraModel):
        """General evaluation routine."""
        with self.lock:
            use_post_rejection = self.use_post_rejection.value
            plot_calibration_data_statistics = self.plot_calibration_data_statistics.value
            plot_calibration_results_statistics = self.plot_calibration_results_statistics.value

        t0 = time.time()

        raw_training_detections = data_collector.get_training_detections()
        raw_evaluation_detections = data_collector.get_evaluation_detections()

        num_training_detections = len(raw_training_detections)
        num_evaluation_detections = len(raw_evaluation_detections)

        if use_post_rejection:
            training_post_rejection_inliers = self._post_rejection_impl(
                raw_training_detections, evaluation_model
            )
            evaluation_post_rejection_inliers = self._post_rejection_impl(
                raw_evaluation_detections, evaluation_model
            )

        else:
            training_post_rejection_inliers = raw_training_detections
            evaluation_post_rejection_inliers = raw_evaluation_detections

        training_num_post_rejection_inliers = len(training_post_rejection_inliers)
        evaluation_num_post_rejection_inliers = len(evaluation_post_rejection_inliers)

        tf = time.time()
        dt = tf - t0

        training_reprojection_errors = [
            evaluation_model.get_reprojection_errors(detection)
            for detection in raw_training_detections
        ]
        training_inlier_reprojection_errors = [
            evaluation_model.get_reprojection_errors(detection)
            for detection in training_post_rejection_inliers
        ]

        evaluation_reprojection_errors = [
            evaluation_model.get_reprojection_errors(detection)
            for detection in raw_evaluation_detections
        ]
        evaluation_inlier_reprojection_errors = [
            evaluation_model.get_reprojection_errors(detection)
            for detection in evaluation_post_rejection_inliers
        ]

        training_rms_error = (
            np.sqrt(np.power(np.concatenate(training_reprojection_errors, axis=0), 2).mean())
            if len(training_reprojection_errors) > 0
            else np.inf
        )
        training_inlier_rms_error = (
            np.sqrt(np.power(np.concatenate(training_inlier_reprojection_errors, axis=0), 2).mean())
            if len(training_inlier_reprojection_errors) > 0
            else np.inf
        )

        evaluation_rms_error = (
            np.sqrt(np.power(np.concatenate(evaluation_reprojection_errors, axis=0), 2).mean())
            if len(evaluation_reprojection_errors) > 0
            else np.inf
        )
        evaluation_inlier_rms_error = (
            np.sqrt(
                np.power(np.concatenate(evaluation_inlier_reprojection_errors, axis=0), 2).mean()
            )
            if len(evaluation_inlier_reprojection_errors) > 0
            else np.inf
        )

        if plot_calibration_data_statistics:
            self._plot_calibration_data_statistics_impl(
                evaluation_model,
                raw_training_detections,
                raw_training_detections,
                raw_training_detections,
                training_post_rejection_inliers,
                raw_evaluation_detections,
            )

        if plot_calibration_results_statistics:
            self._plot_calibration_results_statistics(
                evaluation_model,
                raw_training_detections,
                training_post_rejection_inliers,
                raw_evaluation_detections,
            )

        self.evaluation_results_signal.emit(
            dt,
            num_training_detections,
            training_num_post_rejection_inliers,
            training_rms_error,
            training_inlier_rms_error,
            num_evaluation_detections,
            evaluation_num_post_rejection_inliers,
            evaluation_rms_error,
            evaluation_inlier_rms_error,
        )

    def _calibrate_fast(self, data_collector: DataCollector, camera_model: CameraModel):
        """Fast calibration routine."""
        with self.lock:
            max_samples = self.max_fast_calibration_samples.value

        raw_training_detections = data_collector.get_training_detections()

        if len(raw_training_detections) > max_samples:
            calibration_training_detections = self._random_subsampling_impl(
                raw_training_detections, max_samples
            )
        else:
            calibration_training_detections = raw_training_detections

        new_camera_model = self._calibration_impl(calibration_training_detections)

        initial_reprojection_errors = (
            [
                camera_model.get_reprojection_errors(detection)
                for detection in raw_training_detections
            ]
            if camera_model is not None
            else [np.array([np.inf, np.inf]).reshape(1, 2) for _ in raw_training_detections]
        )

        new_reprojection_errors = (
            [
                new_camera_model.get_reprojection_errors(detection)
                for detection in raw_training_detections
            ]
            if new_camera_model is not None
            else [np.inf for _ in raw_training_detections]
        )

        initial_rms_error = (
            np.sqrt(np.power(np.concatenate(initial_reprojection_errors, axis=0), 2).mean())
            if len(initial_reprojection_errors) > 0
            else np.inf
        )

        new_rms_error = (
            np.sqrt(np.power(np.concatenate(new_reprojection_errors, axis=0), 2).mean())
            if len(new_reprojection_errors) > 0
            else np.inf
        )

        final_camera_model = new_camera_model if new_rms_error < initial_rms_error else camera_model

        self.partial_calibration_results_signal.emit(final_camera_model)

    def _pre_rejection_filter_impl(
        self, detections: List[BoardDetection]
    ) -> Tuple[CameraModel, List[BoardDetection]]:
        """Implement a rejection filter via ransac."""
        with self.lock:
            pre_rejection_min_hypotheses = self.pre_rejection_min_hypotheses.value
            pre_rejection_iterations = self.pre_rejection_iterations.value
            pre_rejection_max_rms_error = self.pre_rejection_max_rms_error.value

        min_error = np.inf
        max_inliers = 0
        best_inlier_mask = None

        num_detections = len(detections)
        min_samples = pre_rejection_min_hypotheses

        if num_detections < min_samples:
            return None, detections

        height = detections[0].get_image_height()
        width = detections[0].get_image_width()

        for it in range(pre_rejection_iterations):
            indexes = np.random.choice(num_detections, min_samples, replace=False)
            sampled_detections = [detections[i] for i in indexes]

            sampled_object_points = [
                detection.get_flattened_object_points() for detection in sampled_detections
            ]
            sampled_image_points = [
                detection.get_flattened_image_points() for detection in sampled_detections
            ]

            model = CameraModel()
            model.calibrate(
                height=height,
                width=width,
                object_points_list=sampled_object_points,
                image_points_list=sampled_image_points,
            )

            rms_errors = np.array(
                [model.get_reprojection_rms_error(detection) for detection in detections]
            )
            inlier_mask = rms_errors < pre_rejection_max_rms_error
            num_inliers = inlier_mask.sum()

            print(
                f"Iteration {it}: inliers: {num_inliers} | mean rms: {rms_errors.mean():.2f} | min rms: {rms_errors.min():.2f} | max rms: {rms_errors.max():.2f}"
            )

            if num_inliers > max_inliers or (
                num_inliers == max_inliers and rms_errors.mean() < min_error
            ):
                best_inlier_mask = inlier_mask
                min_error = rms_errors.mean()
                max_inliers = num_inliers

        print(
            f"Pre rejection inliers = {max_inliers}/{len(detections)} | threshold = {pre_rejection_max_rms_error:.2f}"
        )

        return model, [detections[i] for i in best_inlier_mask.nonzero()[0]]

    def _entropy_maximization_subsampling_impl(
        self, detections: List[BoardDetection], camera_model: CameraModel
    ) -> List[BoardDetection]:
        """Subsample detections via greedy entropy maximization."""
        with self.lock:
            subsampling_pixel_cells = self.subsampling_pixel_cells.value
            subsampling_max_tilt_deg = self.subsampling_max_tilt_deg.value
            subsampling_tilt_resolution = self.subsampling_tilt_resolution.value
            max_calibration_samples = self.max_calibration_samples.value

        pixel_cells: int = subsampling_pixel_cells
        max_tilt_deg: float = subsampling_max_tilt_deg
        tilt_cells: int = int(2 * np.ceil(max_tilt_deg / subsampling_tilt_resolution))

        num_detections = len(detections)
        accepted_array = np.zeros((num_detections,), dtype=np.bool_)

        accepted_pixel_occupancy = np.zeros((pixel_cells, pixel_cells))
        accepted_tilt_occupancy = np.zeros((2 * tilt_cells, 2 * tilt_cells))

        max_pixel_bits = 2 * np.log2(pixel_cells)
        max_tilt_bits = 2 * np.log2(2 * tilt_cells)

        # add random initial detection
        initial_idx = np.random.randint(0, num_detections)
        accepted_array[initial_idx] = True

        add_detection(
            detections[initial_idx],
            camera_model,
            accepted_pixel_occupancy,
            accepted_tilt_occupancy,
            pixel_cells,
            tilt_cells,
            max_tilt_deg,
        )

        # add detections greedily
        for it in range(0, max_calibration_samples - 1):

            current_entropy = (get_entropy(accepted_pixel_occupancy) / max_pixel_bits) + (
                get_entropy(accepted_tilt_occupancy) / max_tilt_bits
            )

            max_delta_entropy = -np.inf
            max_delta_entropy_idx = -1

            for idx in np.logical_not(accepted_array).nonzero()[0]:

                pixel_occupancy = np.copy(accepted_pixel_occupancy)
                tilt_occupancy = np.copy(accepted_tilt_occupancy)

                add_detection(
                    detections[idx],
                    camera_model,
                    pixel_occupancy,
                    tilt_occupancy,
                    pixel_cells,
                    tilt_cells,
                    max_tilt_deg,
                )
                entropy = (get_entropy(pixel_occupancy) / max_pixel_bits) + (
                    get_entropy(tilt_occupancy) / max_tilt_bits
                )
                delta_entropy = entropy - current_entropy

                if delta_entropy > max_delta_entropy:
                    max_delta_entropy = delta_entropy
                    max_delta_entropy_idx = idx

            print(f"iteration={it}: delta entropy={max_delta_entropy:.3f}")
            accepted_array[max_delta_entropy_idx] = True
            add_detection(
                detections[max_delta_entropy_idx],
                camera_model,
                accepted_pixel_occupancy,
                accepted_tilt_occupancy,
                pixel_cells,
                tilt_cells,
                max_tilt_deg,
            )

        return [detections[i] for i in accepted_array.nonzero()[0]]

    def _random_subsampling_impl(
        self, detections: List[BoardDetection], max_samples
    ) -> List[BoardDetection]:
        """Subsample the detections uniformly."""
        indexes = np.random.choice(len(detections), max_samples, replace=False)
        return [detections[i] for i in indexes]

    def _post_rejection_impl(
        self, detections: List[BoardDetection], model: CameraModel
    ) -> List[BoardDetection]:
        with self.lock:
            post_rejection_max_rms_error = self.post_rejection_max_rms_error.value

        rms_error = np.array(
            [model.get_reprojection_rms_error(detection) for detection in detections]
        )
        inliers_mask = rms_error < post_rejection_max_rms_error

        print(
            f"Post rejection inliers = {inliers_mask.sum()}/{len(detections)} | threshold = {post_rejection_max_rms_error}"
        )

        return [detections[i] for i in inliers_mask.nonzero()[0]]

    def _plot_calibration_data_statistics_impl(
        self,
        calibrated_model: CameraModel,
        training_detections: List[BoardDetection],
        pre_rejection_inlier_detections: List[BoardDetection],
        subsampled_detections: List[BoardDetection],
        post_rejection_inlier_detections: List[BoardDetection],
        evaluation_detections: List[BoardDetection],
    ):
        """Plot the statistics of the different set of data involved in the calibration. Since the implementation is in matplotlib it requires to be done in another process."""
        with self.lock:
            viz_pixel_cells = self.viz_pixel_cells.value
            viz_tilt_resolution = self.viz_tilt_resolution.value
            viz_max_tilt_deg = self.viz_max_tilt_deg.value
            viz_z_cells = self.viz_z_cells.value

        plot_process = mp.Process(
            target=_plot_calibration_data_statistics,
            args=(
                calibrated_model,
                training_detections,
                pre_rejection_inlier_detections,
                subsampled_detections,
                post_rejection_inlier_detections,
                evaluation_detections,
                viz_pixel_cells,
                viz_tilt_resolution,
                viz_max_tilt_deg,
                viz_z_cells,
            ),
            daemon=True,
        )
        plot_process.start()

    def _plot_calibration_results_statistics(
        self,
        calibrated_model: CameraModel,
        training_detections: List[BoardDetection],
        inlier_detections: List[BoardDetection],
        evaluation_detections: List[BoardDetection],
    ):
        """Plot the calibration result statistics of the different set of data involved in the calibration. Since the implementation is in matplotlib it requires to be done in another process."""
        with self.lock:
            viz_pixel_cells = self.viz_pixel_cells.value
            viz_tilt_resolution = self.viz_tilt_resolution.value
            viz_max_tilt_deg = self.viz_max_tilt_deg.value
            viz_z_cells = self.viz_z_cells.value

        plot_process = mp.Process(
            target=_plot_calibration_results_statistics,
            args=(
                calibrated_model,
                training_detections,
                inlier_detections,
                evaluation_detections,
                viz_pixel_cells,
                viz_tilt_resolution,
                viz_max_tilt_deg,
                viz_z_cells,
            ),
            daemon=True,
        )
        plot_process.start()

    def _calibration_impl(self, detections: List[BoardDetection]) -> CameraModel:
        """Actual implementation of the calibration. Since this is an abstract class it remains unimplemented."""
        raise NotImplementedError
