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


from collections import defaultdict

from dt_apriltags import Detector as _Detector
from intrinsic_camera_calibrator.board_detections.apriltag_grid_detection import (
    ApriltagGridDetection,
)
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.utils import to_grayscale


class Detector(_Detector):
    def __del__(self):
        if self.tag_detector_ptr is not None:
            # destroy the detector
            self.libc.apriltag_detector_destroy.restype = None  # cSpell:ignore libc
            self.libc.apriltag_detector_destroy(self.tag_detector_ptr)

            # destroy the tag families
            for family, tf in self.tag_families.items():
                if "tag16h5" == family:
                    self.libc.tag16h5_destroy.restype = None
                    self.libc.tag16h5_destroy(tf)
                elif "tag25h9" == family:
                    self.libc.tag25h9_destroy.restype = None
                    self.libc.tag25h9_destroy(tf)
                elif "tag36h11" == family:
                    self.libc.tag36h11_destroy.restype = None
                    self.libc.tag36h11_destroy(tf)
                elif "tagCircle21h7" == family:
                    self.libc.tagCircle21h7_destroy.restype = None
                    self.libc.tagCircle21h7_destroy(tf)
                elif "tagCircle49h12" == family:
                    self.libc.tagCircle49h12_destroy.restype = None
                    self.libc.tagCircle49h12_destroy(tf)
                elif "tagCustom48h12" == family:
                    self.libc.tagCustom48h12_destroy.restype = None
                    self.libc.tagCustom48h12_destroy(tf)
                elif "tagStandard41h12" == family:
                    self.libc.tagStandard41h12_destroy.restype = None
                    self.libc.tagStandard41h12_destroy(tf)
                elif "tagStandard52h13" == family:
                    self.libc.tagStandard52h13_destroy.restype = None
                    self.libc.tagStandard52h13_destroy(tf)


class ApriltagGridDetector(BoardDetector):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        # cSpell:ignore nthreads
        self.nthreads = Parameter(int, value=8, min_value=1, max_value=16)
        self.quad_decimate = Parameter(int, value=1, min_value=0, max_value=8)
        self.quad_sigma = Parameter(float, value=1.5, min_value=0.0, max_value=10.0)
        self.refine_edges = Parameter(bool, value=1, min_value=0, max_value=1)
        self.decode_sharpening = Parameter(float, value=0.25, min_value=0.0, max_value=10.0)
        self.debug = Parameter(bool, value=0, min_value=0, max_value=1)

        self.max_hamming_error = Parameter(int, value=0, min_value=0, max_value=2)
        self.min_margin = Parameter(float, value=25.0, min_value=0.0, max_value=1000.0)
        self.min_detection_ratio = Parameter(float, value=0.2, min_value=0.05, max_value=0.5)

        self.detector = None
        self.current_nthreads = None
        self.current_quad_decimate = None
        self.current_quad_sigma = None
        self.current_refine_edges = None
        self.current_decode_sharpening = None
        self.current_debug = None
        pass

    def detect(self, img):
        """Slot to detect boards from an image. Results are sent through the detection_results signals."""
        if img is None:
            self.detection_results_signal.emit(None, None)
            return

        with self.lock:
            min_margin = self.min_margin.value
            max_hamming = self.max_hamming_error.value
            min_detection_ratio = self.min_detection_ratio.value
            (cols, rows) = (self.board_parameters.cols.value, self.board_parameters.rows.value)
            min_index = self.board_parameters.min_index.value
            tag_size = self.board_parameters.tag_size.value
            tag_spacing = self.board_parameters.tag_spacing.value
            h, w = img.shape[0:2]

            # Rebuild the detector if needed
            if (
                self.nthreads.value != self.current_nthreads
                or self.quad_decimate.value != self.current_quad_decimate
                or self.quad_sigma.value != self.current_quad_sigma
                or self.refine_edges.value != self.current_refine_edges
                or self.decode_sharpening.value != self.current_decode_sharpening
                or self.debug.value != self.current_debug
            ):
                self.current_nthreads = self.nthreads.value
                self.current_quad_decimate = self.quad_decimate.value
                self.current_quad_sigma = self.quad_sigma.value
                self.current_refine_edges = self.refine_edges.value
                self.current_decode_sharpening = self.decode_sharpening.value
                self.current_debug = self.debug.value

                self.detector = Detector(
                    families="tag16h5",
                    nthreads=self.current_nthreads,
                    quad_decimate=self.current_quad_decimate,
                    quad_sigma=self.current_quad_sigma,
                    refine_edges=self.current_refine_edges,
                    decode_sharpening=self.current_decode_sharpening,
                    debug=self.current_debug,
                )

        grayscale = to_grayscale(img)

        tags = self.detector.detect(grayscale, False, None, None)
        tags = [
            tag
            for tag in tags
            if tag.decision_margin > min_margin
            and tag.hamming <= max_hamming
            and tag.tag_id < min_index + rows * cols
            and tag.tag_id >= min_index
        ]

        id_dict = defaultdict(int)

        for tag in tags:
            id_dict[tag.tag_id] += 1

        tags = [tag for tag in tags if id_dict[tag.tag_id] == 1]
        tags.sort(key=lambda tag: tag.tag_id)

        if len(tags) < rows * cols * min_detection_ratio:
            self.detection_results_signal.emit(img, None)
            return

        detection = ApriltagGridDetection(
            height=h,
            width=w,
            rows=rows,
            cols=cols,
            tag_size=tag_size,
            tag_spacing=tag_spacing,
            min_index=min_index,
            tags=tags,
        )

        self.detection_results_signal.emit(img, detection)
