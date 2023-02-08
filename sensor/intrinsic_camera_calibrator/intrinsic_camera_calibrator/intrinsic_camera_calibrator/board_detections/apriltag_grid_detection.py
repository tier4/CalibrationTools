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


from collections import defaultdict
from typing import List
from typing import Tuple

import cv2
from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
import numpy as np


class ApriltagGridDetection(BoardDetection):
    def __init__(
        self,
        height: int,
        width: int,
        rows: int,
        cols: int,
        tag_size: float,
        tag_spacing: float,
        tags,
    ):
        super().__init__(height=height, width=width, rows=rows, cols=cols)

        # Compute the object points
        self.tags = tags

        # cSpell:ignore hsize
        hsize = 0.5 * tag_size
        single_object_points = np.array(
            [[-hsize, -hsize, 0.0], [hsize, -hsize, 0.0], [hsize, hsize, 0.0], [-hsize, hsize, 0.0]]
        )

        self.ordered_object_points = []
        self.ordered_image_points = []

        scan_lines_dict = defaultdict(list)
        factor = tag_size * (1.0 + tag_spacing)

        for tag in self.tags:
            row = tag.tag_id // cols
            col = tag.tag_id % cols
            x = (col - 0.5 * (cols - 1)) * factor
            y = (row - 0.5 * (rows - 1)) * factor
            object_points = single_object_points + np.array([[x, y, 0.0]])

            self.ordered_object_points.append(object_points)
            self.ordered_image_points.append(tag.corners)

            scan_lines_dict[row].append(tag.corners[0])
            scan_lines_dict[row].append(tag.corners[1])
            scan_lines_dict[rows + row].append(tag.corners[2])
            scan_lines_dict[rows + row].append(tag.corners[3])

        self.scan_lines = list(scan_lines_dict.values())

        homography, _ = cv2.findHomography(
            self.get_flattened_object_points()[:, 0:2], self.get_flattened_image_points()
        )

        self.object_external_corners = np.array(
            [
                [-0.5 * ((cols - 1) * factor + tag_size), -0.5 * ((rows - 1) * factor + tag_size)],
                [0.5 * ((cols - 1) * factor + tag_size), -0.5 * ((rows - 1) * factor + tag_size)],
                [0.5 * ((cols - 1) * factor + tag_size), 0.5 * ((rows - 1) * factor + tag_size)],
                [-0.5 * ((cols - 1) * factor + tag_size), 0.5 * ((rows - 1) * factor + tag_size)],
            ]
        )

        self.image_external_corners = cv2.perspectiveTransform(
            self.object_external_corners.reshape(1, -1, 2), homography
        )[0]

    def get_ordered_object_points(self) -> List[np.array]:
        """Return the object points of the board ordered as a list of arrays."""
        return self.ordered_object_points

    def get_ordered_image_points(self) -> List[np.array]:
        """Return the image points of the board ordered as a list of arrays."""
        return self.ordered_image_points

    def get_flattened_object_points(self) -> np.array:
        """Return the object points of the board as a (N, 3) array."""
        return np.array(self.ordered_object_points).reshape(-1, 3)

    def get_flattened_image_points(self) -> np.array:
        """Return the image points of the board as a (N, 2) array."""
        return np.array(self.ordered_image_points).reshape(-1, 2)

    def get_linear_error_rms(self) -> float:
        def squared_error(p, p1, p2):
            p = p - p1
            p2 = p2 - p1
            p2 /= np.linalg.norm(p2)
            return np.abs(np.power(np.linalg.norm(p), 2) - np.power(np.dot(p, p2), 2))

        error = 0
        n = 0

        if self.scan_lines is None or len(self.scan_lines) == 0:
            return np.inf

        for scan_line in self.scan_lines:

            p1 = scan_line[0]
            p2 = scan_line[-1]

            for i in range(1, len(scan_line) - 1):
                p = scan_line[i]
                error += squared_error(p, p1, p2)
                n += 1

        # There are cases where the linear error can not be computed due to having only 2 points per scan line
        return np.sqrt(error / n) if n > 0 else np.inf

    def _get_border_image_points(self) -> Tuple[np.array, np.array, np.array, np.array]:
        return (
            self.image_external_corners[0],
            self.image_external_corners[1],
            self.image_external_corners[2],
            self.image_external_corners[3],
        )

    def get_flattened_cell_sizes(self):
        if self._cached_flattened_cell_sizes is not None:
            return self._cached_flattened_cell_sizes

        cell_sizes = []

        for image_points in self.ordered_image_points:
            a1 = image_points
            a2 = np.concatenate([image_points[1:], image_points[0:1]], axis=0)

            edge_dists = np.linalg.norm(a1 - a2, axis=-1)
            cell_size = edge_dists.mean()
            cell_sizes.extend([cell_size, cell_size, cell_size, cell_size])

        return np.array(cell_sizes)
