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

import cv2
from intrinsic_camera_calibrator.board_detections.chess_board_detection import ChessBoardDetection
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.utils import to_grayscale
import numpy as np


class ChessBoardDetector(BoardDetector):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.adaptive_thresh = Parameter(bool, value=True, min_value=False, max_value=True)
        self.normalize_image = Parameter(bool, value=True, min_value=False, max_value=True)
        self.fast_check = Parameter(bool, value=True, min_value=False, max_value=True)
        self.refine = Parameter(bool, value=True, min_value=False, max_value=True)
        pass

    def detect(self, img):
        """Slot to detect boards from an image. Results are sent through the detection_results signals."""
        if img is None:
            self.detection_results_signal.emit(None, None)
            return

        with self.lock:
            h, w = img.shape[0:2]
            (cols, rows) = (self.board_parameters.cols.value, self.board_parameters.rows.value)
            cell_size = self.board_parameters.cell_size.value

            flags = 0
            flags |= cv2.CALIB_CB_ADAPTIVE_THRESH if self.adaptive_thresh.value else 0
            flags |= cv2.CALIB_CB_NORMALIZE_IMAGE if self.normalize_image.value else 0
            flags |= cv2.CALIB_CB_FAST_CHECK if self.fast_check.value else 0
            refine = self.refine.value

        grayscale = to_grayscale(img)

        (ok, corners) = cv2.findChessboardCorners(grayscale, (cols, rows), flags=flags)

        if not ok:
            self.detection_results_signal.emit(img, None)
            return

        if ok and refine:

            dist_matrix = np.linalg.norm(
                corners.reshape(-1, 1, 2) - corners.reshape(1, -1, 2), axis=-1
            )
            np.fill_diagonal(dist_matrix, np.inf)
            min_distance = dist_matrix.min()
            radius = int(np.ceil(min_distance * 0.5))

            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(grayscale, corners, (radius, radius), (-1, -1), criteria)

        image_points = corners.reshape((rows, cols, 2))
        x_array = cell_size * (np.array(range(cols)) - 0.5 * cols)
        y_array = cell_size * (np.array(range(rows)) - 0.5 * rows)
        object_points = np.stack([*np.meshgrid(x_array, y_array), np.zeros((rows, cols))], axis=-1)

        detection = ChessBoardDetection(
            height=h,
            width=w,
            rows=rows,
            cols=cols,
            object_points=object_points,
            image_points=image_points,
        )

        self.detection_results_signal.emit(img, detection)
