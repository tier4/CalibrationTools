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
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.parameter import Parameter


class ChessBoardDetector(BoardDetector):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.chessboard_int_param = Parameter(int, value=1, min_value=1, max_value=4)
        self.chessboard_float_param = Parameter(float, value=0.0, min_value=0.0, max_value=1.0)
        self.chessboard_bool_param = Parameter(bool, value=False, min_value=False, max_value=True)

        self.adaptive_thresh = Parameter(bool, value=True, min_value=False, max_value=True)
        self.normalize_image = Parameter(bool, value=True, min_value=False, max_value=True)
        self.fast_check = Parameter(bool, value=True, min_value=False, max_value=True)
        pass

    def detect(self, img):
        """Slot to detect boards from an image. Results are sent through the detection_results signals."""
        flags = 0
        flags |= cv2.CALIB_CB_ADAPTIVE_THRESH if self.adaptive_thresh.value else 0
        flags |= cv2.CALIB_CB_NORMALIZE_IMAGE if self.normalize_image.value else 0
        flags |= cv2.CALIB_CB_FAST_CHECK if self.fast_check.value else 0

        mono = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (ok, corners) = cv2.findChessboardCorners(
            mono, (self.board_parameters.cols.value, self.board_parameters.rows.value), flags=flags
        )

        if ok:
            cv2.drawChessboardCorners(
                img,
                (self.board_parameters.cols.value, self.board_parameters.rows.value),
                corners,
                ok,
            )

        self.detection_results.emit(img, corners, corners, 0.0, 0.0, 0.0)
