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

# cSpell:enableCompoundWords
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

        self.resized_detection = Parameter(bool, value=True, min_value=False, max_value=True)
        self.resized_max_resolution = Parameter(int, value=2000, min_value=500, max_value=5000)

    def detect(self, img: np.array, stamp: float):
        """Slot to detect boards from an image. Results are sent through the detection_results signals."""
        if img is None:
            self.detection_results_signal.emit(None, None)
            return

        with self.lock:
            h, w = img.shape[0:2]
            (cols, rows) = (self.board_parameters.cols.value, self.board_parameters.rows.value)
            cell_size = self.board_parameters.cell_size.value

            filter_by_area = self.filter_by_area.value
            min_area_percentage = self.min_area_percentage.value
            max_area_percentage = self.max_area_percentage.value
            min_dist_between_blobs_percentage = self.min_dist_between_blobs_percentage.value

            flags = 0
            flags |= cv2.CALIB_CB_CLUSTERING if self.clustering.value else 0
            flags |= (
                cv2.CALIB_CB_SYMMETRIC_GRID
                if self.symmetric_grid.value
                else cv2.CALIB_CB_ASYMMETRIC_GRID
            )

            resized_detection = self.resized_detection.value
            resized_max_resolution = self.resized_max_resolution.value

        # Find the resized dimensions
        ratio = float(w) / float(h)

        if w > h:
            resized_w = int(resized_max_resolution)
            resized_h = int(resized_max_resolution / ratio)
        else:
            resized_w = int(resized_max_resolution * ratio)
            resized_h = int(resized_max_resolution)

        # Setting blob detector
        full_res_params = cv2.SimpleBlobDetector_Params()
        full_res_params.filterByArea = filter_by_area
        full_res_params.minArea = min_area_percentage * h * w / 100.0
        full_res_params.maxArea = max_area_percentage * h * w / 100.0
        full_res_params.minDistBetweenBlobs = min_dist_between_blobs_percentage * max(h, w) / 100.0

        resized_params = cv2.SimpleBlobDetector_Params()
        resized_params.filterByArea = filter_by_area
        resized_params.minArea = min_area_percentage * resized_h * resized_w / 100.0
        resized_params.maxArea = max_area_percentage * resized_h * resized_w / 100.0
        resized_params.minDistBetweenBlobs = (
            min_dist_between_blobs_percentage * max(resized_h, resized_w) / 100.0
        )

        full_res_detector = cv2.SimpleBlobDetector_create(full_res_params)
        resized_detector = cv2.SimpleBlobDetector_create(resized_params)

        grayscale = to_grayscale(img)

        def detect(detection_image, detector):
            (ok, corners) = cv2.findCirclesGrid(
                detection_image, (cols, rows), flags=flags, blobDetector=detector
            )

            if not ok:
                (ok, corners) = cv2.findCirclesGrid(
                    detection_image, (cols, rows), flags=flags, blobDetector=detector
                )

                # we need to swap the axes of the detections back to make it consistent
                if ok:
                    corners_2d_array = corners.reshape((cols, rows, 2))
                    corners_transposed = np.transpose(corners_2d_array, (1, 0, 2))
                    corners = corners_transposed.reshape(-1, 1, 2)

            return (ok, corners)

        if not resized_detection or max(h, w) <= resized_max_resolution:
            (ok, corners) = detect(grayscale, full_res_detector)

        else:
            # Resize
            resized = cv2.resize(img, (resized_w, resized_h), interpolation=cv2.INTER_NEAREST)

            # Run the detector on the resized image
            (ok, resized_corners) = detect(resized, resized_detector)

            if not ok:
                self.detection_results_signal.emit(img, None, stamp)
                return

            # Re escalate the corners
            corners = resized_corners * np.array(
                [float(w) / resized_w, float(h) / resized_h], dtype=np.float32
            )

            # Estimate the ROI in the original image
            border = max(
                corners[:, 0, 0].max() - corners[:, 0, 0].min(),
                corners[:, 0, 1].max() - corners[:, 0, 1].min(),
            ) / min(cols, rows)

            roi_min_j = int(max(0, corners[:, 0, 1].min() - border))
            roi_min_i = int(max(0, corners[:, 0, 0].min() - border))
            roi_max_j = int(min(w, corners[:, 0, 1].max() + border))
            roi_max_i = int(min(w, corners[:, 0, 0].max() + border))

            # Extract the ROI of the original image
            roi = grayscale[roi_min_j:roi_max_j, roi_min_i:roi_max_i]

            # Run the detector again
            (ok, roi_corners) = detect(roi, full_res_detector)

            if not ok:
                self.detection_results_signal.emit(img, None, stamp)
                return

            # Re escalate the coordinates
            corners = roi_corners + np.array([roi_min_i, roi_min_j], dtype=np.float32)

        # reverse the corners if needed
        if np.linalg.norm(corners[0]) > np.linalg.norm(corners[1]):
            corners = np.flip(corners, axis=0)

        image_points = corners.reshape((rows, cols, 2))
        x_array = cell_size * (np.array(range(cols)) - 0.5 * cols)
        y_array = cell_size * (np.array(range(rows)) - 0.5 * rows)
        object_points = np.stack([*np.meshgrid(x_array, y_array), np.zeros((rows, cols))], axis=-1)

        detection = DotBoardDetection(
            height=h,
            width=w,
            rows=rows,
            cols=cols,
            object_points=object_points,
            image_points=image_points,
        )

        self.detection_results_signal.emit(img, detection, stamp)
