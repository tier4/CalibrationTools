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


from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.parameter import Parameter


class ApriltagGridDetector(BoardDetector):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.quad_decimate = Parameter(int, value=1, min=1, max=4)
        self.quad_sigma = Parameter(float, value=0.0, min=0.0, max=1.0)
        pass

    def detect(self, img):
        return super().detect(img)
