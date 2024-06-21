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

from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.parameter import ParameterizedClass


class ApriltagGridParameters(ParameterizedClass):
    def __init__(self, cfg):
        self.rows = Parameter(int, value=1, min_value=1, max_value=20)
        self.cols = Parameter(int, value=1, min_value=1, max_value=20)
        self.tag_size = Parameter(float, value=0.2, min_value=0.01, max_value=1.0)
        self.tag_spacing = Parameter(float, value=0.25, min_value=0.0, max_value=1.0)
        self.min_index = Parameter(int, value=0, min_value=0, max_value=32)

        super().__init__(cfg=cfg)
