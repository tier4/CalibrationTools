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


class BoardEnum(Enum):
    CHESS_BOARD = {"name": "chess_board", "display": "Chess board"}
    DOT_BOARD = {"name": "dot_board", "display": "Dot board"}
    APRILTAG_GRID = {"name": "apriltag_grid", "display": "Apriltag grid"}

    def from_name(name: str) -> "BoardEnum":
        for board in BoardEnum:
            if name == board.value["name"]:
                return board

        raise ValueError

    def from_index(i: int):
        return list(BoardEnum)[i]

    def get_id(self) -> int:
        for index, board in enumerate(BoardEnum):
            if board == self:
                return index

        raise ValueError
