from typing import List
from typing import Tuple

# import cv2
from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection

# from intrinsic_camera_calibrator.boards import BoardEnum
import numpy as np


class DotBoardDetection(BoardDetection):
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

    def get_normalized_skew(self) -> float:
        def angle(a, b, c):
            """Return angle between lines ab, bc."""
            ab = a - b
            cb = c - b
            return np.arccos(np.dot(ab, cb) / (np.linalg.norm(ab) * np.linalg.norm(cb)))

        if self._cached_normalized_skew is not None:
            return self._cached_normalized_skew

        up_left, up_right, down_right, down_left = self._get_border_image_points()

        a012 = angle(up_left, up_right, down_right)
        a123 = angle(up_right, down_right, down_left)
        a230 = angle(down_right, down_left, up_left)
        a301 = angle(down_left, up_left, up_right)

        self._cached_normalized_skew = (
            sum([min(1.0, 2.0 * abs((np.pi / 2.0) - angle)) for angle in [a012, a123, a230, a301]])
            / 4
        )
        return self._cached_normalized_skew

    def get_normalized_size(self) -> float:

        if self._cached_normalized_size is not None:
            return self._cached_normalized_size

        (up_left, up_right, down_right, down_left) = self._get_border_image_points()
        a = up_right - up_left
        b = down_right - up_right
        c = down_left - down_right
        p = b + c
        q = a + b

        # The sqrt is to assign more "resolution" to close distances
        self._cached_normalized_size = np.sqrt(
            np.abs(p[0] * q[1] - p[1] * q[0]) / (2.0 * self.height * self.width)
        )
        return self._cached_normalized_size

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
