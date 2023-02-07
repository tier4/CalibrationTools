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
from intrinsic_camera_calibrator.board_detections.dotboard_detection import DotBoardDetection
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.utils import to_grayscale
import numpy as np


class DotBoardDetector(BoardDetector):
    """Detector class for a/symmetric circle/dot boards."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.symmetric_grid = Parameter(bool, value=True, min_value=False, max_value=True)
        self.clustering = Parameter(bool, value=True, min_value=False, max_value=True)

        self.filter_by_area = Parameter(bool, value=True, min_value=False, max_value=True)
        self.min_area_percentage = Parameter(float, value=0.01, min_value=0.001, max_value=0.1)
        self.max_area_percentage = Parameter(float, value=1.2, min_value=0.1, max_value=10.0)
        self.min_dist_between_blobs_percentage = Parameter(
            float, value=1.0, min_value=0.1, max_value=10.0
        )
        pass

    def detect(self, img: np.array):
        """Slot to detect boards from an image. Results are sent through the detection_results signals."""
        if img is None:
            self.detection_results_signal.emit(None, None)
            return

        with self.lock:
            h, w = img.shape[0:2]
            (cols, rows) = (self.board_parameters.cols.value, self.board_parameters.rows.value)
            cell_size = self.board_parameters.cell_size.value

            # Setting blob detector
            params = cv2.SimpleBlobDetector_Params()
            params.filterByArea = self.filter_by_area.value
            params.minArea = self.min_area_percentage.value * h * w / 100.0
            params.maxArea = self.max_area_percentage.value * h * w / 100.0
            params.minDistBetweenBlobs = (
                self.min_dist_between_blobs_percentage.value * max(h, w) / 100.0
            )

            detector = cv2.SimpleBlobDetector_create(params)

            flags = 0
            flags |= cv2.CALIB_CB_CLUSTERING if self.clustering.value else 0
            flags |= (
                cv2.CALIB_CB_SYMMETRIC_GRID
                if self.symmetric_grid.value
                else cv2.CALIB_CB_ASYMMETRIC_GRID
            )

        grayscale = to_grayscale(img)

        (ok, corners) = cv2.findCirclesGrid(
            grayscale, (cols, rows), flags=flags, blobDetector=detector
        )

        if not ok:
            (ok, corners) = cv2.findCirclesGrid(
                grayscale, (cols, rows), flags=flags, blobDetector=detector
            )

            # we need to swap the axes of the detections back to make it consistent
            if ok:
                corners_2d_array = corners.reshape((cols, rows, 2))
                corners_transposed = np.transpose(corners_2d_array, (1, 0, 2))
                corners = corners_transposed.reshape(-1, 1, 2)

        if not ok:
            self.detection_results_signal.emit(img, None)
            return

        # reverse the corners if needed
        if np.linalg.norm(corners[0]) > np.linalg.norm(corners[1]):
            corners = np.flip(corners, axis=0)

        image_points = corners.reshape((rows, cols, 2))
        xarray = cell_size * (np.array(range(cols)) - 0.5 * cols)
        yarray = cell_size * (np.array(range(rows)) - 0.5 * rows)
        object_points = np.stack([*np.meshgrid(xarray, yarray), np.zeros((rows, cols))], axis=-1)

        detection = DotBoardDetection(
            height=h,
            width=w,
            rows=rows,
            cols=cols,
            object_points=object_points,
            image_points=image_points,
        )

        self.detection_results_signal.emit(img, detection)
