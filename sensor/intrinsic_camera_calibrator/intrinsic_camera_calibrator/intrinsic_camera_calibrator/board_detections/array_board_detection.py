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


from typing import List
from typing import Tuple

from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
import numpy as np


class ArrayBoardDetection(BoardDetection):
    def __init__(
        self,
        height: int,
        width: int,
        rows: int,
        cols: int,
        object_points: np.array,
        image_points: np.array,
    ):
        super().__init__(height=height, width=width, rows=rows, cols=cols)

        self.object_points = object_points
        self.image_points = image_points

    def _get_border_image_points(self) -> Tuple[np.array, np.array, np.array, np.array]:
        return (
            self.image_points[0, 0],
            self.image_points[0, -1],
            self.image_points[-1, -1],
            self.image_points[-1, 0],
        )

    def get_ordered_object_points(self) -> List[np.array]:
        return [self.object_points[row] for row in range(self.rows)]

    def get_ordered_image_points(self) -> List[np.array]:
        return [self.image_points[row] for row in range(self.rows)]

    def get_flattened_object_points(self) -> np.array:
        return self.object_points.reshape(-1, 3)

    def get_flattened_image_points(self) -> np.array:
        return self.image_points.reshape(-1, 2)

    def get_linear_error_rms(self) -> float:
        def squared_error(p, p1, p2):
            p = p - p1
            p2 = p2 - p1
            p2 /= np.linalg.norm(p2)
            return np.abs(np.power(np.linalg.norm(p), 2) - np.power(np.dot(p, p2), 2))

        if self._cached_linear_error_rms is not None:
            return self._cached_linear_error_rms

        error = 0

        for j in range(self.rows):
            p1 = self.image_points[j, 0]
            p2 = self.image_points[j, -1]

            for i in range(1, self.cols - 1):
                p = self.image_points[j, i]
                error += squared_error(p, p1, p2)

        self._cached_linear_error_rms = np.sqrt(error / (self.rows * (self.cols - 2)))
        return self._cached_linear_error_rms

    def get_flattened_cell_sizes(self):
        if self._cached_flattened_cell_sizes is not None:
            return self._cached_flattened_cell_sizes

        a1 = self.image_points[:, 0:-1, :]
        a2 = self.image_points[:, 1:, :]

        edge_dists = np.linalg.norm(a1 - a2, axis=-1)

        cell_sizes = np.zeros((self.rows, self.cols))
        cell_sizes[:, 0] = edge_dists[:, 0]
        cell_sizes[:, -1] = edge_dists[:, -1]
        cell_sizes[:, 1:-1] = 0.5 * (edge_dists[:, 0:-1] + edge_dists[:, 1:])

        self._cached_flattened_cell_sizes = cell_sizes.flatten()
        return self._cached_flattened_cell_sizes
