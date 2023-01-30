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

from intrinsic_camera_calibrator.board_parameters.apriltag_grid_parameters import (
    ApriltagGridParameters,
)
from intrinsic_camera_calibrator.board_parameters.board_parameters import BoardParameters
from intrinsic_camera_calibrator.boards import BoardEnum
from intrinsic_camera_calibrator.parameter import ParameterizedClass


def make_board_parameters(board_type: BoardEnum, **kwargs) -> ParameterizedClass:
    """Create a board detector using a factory design pattern. Currently we accept a single detector per type of board."""
    classes_dic = {
        BoardEnum.CHESSBOARD: BoardParameters,
        BoardEnum.DOTBOARD: BoardParameters,
        BoardEnum.APRILTAGGRID: ApriltagGridParameters,
    }
    return classes_dic[board_type](**kwargs)
