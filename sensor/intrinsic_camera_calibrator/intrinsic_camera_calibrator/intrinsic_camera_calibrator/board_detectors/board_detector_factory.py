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

# cSpell:enableCompoundWords
from intrinsic_camera_calibrator.board_detectors.apriltag_grid_detector import ApriltagGridDetector
from intrinsic_camera_calibrator.board_detectors.board_detector import BoardDetector
from intrinsic_camera_calibrator.board_detectors.chessboard_detector import ChessBoardDetector
from intrinsic_camera_calibrator.board_detectors.dotboard_detector import DotBoardDetector
from intrinsic_camera_calibrator.boards import BoardEnum


def make_detector(board_type: BoardEnum, **kwargs) -> BoardDetector:
    """Create a board detector using a factory design pattern. Currently we accept a single detector per type of board."""
    classes_dic = {
        BoardEnum.CHESSBOARD: ChessBoardDetector,
        BoardEnum.DOTBOARD: DotBoardDetector,
        BoardEnum.APRILTAGGRID: ApriltagGridDetector,
    }
    return classes_dic[board_type](**kwargs)
