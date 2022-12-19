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


from typing import List
from typing import Optional
from typing import Tuple

import cv2
from intrinsic_camera_calibrator.camera_model import CameraModel
import numpy as np


class BoardDetection:
    """Base class for all board detections. Acts as an abstract class and implements common logic."""

    def __init__(
        self,
        height: Optional[int] = None,
        width: Optional[int] = None,
        rows: Optional[int] = None,
        cols: Optional[int] = None,
    ):

        self.height = height
        self.width = width
        self.rows = rows
        self.cols = cols

        self._precomputed_single_shot_model = None

        self._cached_normalized_skew = None
        self._cached_normalized_size = None
        self._cached_linear_error_rms = None
        self._cached_flattened_cell_sizes = None
        self._cached_center_2d = None

        self._cached_camera_model = None
        self._cached_reprojection_errors = None
        self._cached_tilt = None
        self._cached_rotation_angles = None
        self._cached_pose = None
        self._cached_flattened_3d_points = None

    def _precompute_single_shot_model(self):
        """Compute and caches a camera model calibrated with the current detection."""
        model = CameraModel()
        model.calibrate(
            height=self.height,
            width=self.width,
            object_points_list=[self.get_flattened_object_points()],
            image_points_list=[self.get_flattened_image_points()],
        )
        self._cached_camera_model = model

    def _get_cached_model(self) -> CameraModel:
        """Return the single shot camera model and computes it is has not been pre-computed yet."""
        if self._cached_camera_model is None:
            self._precompute_single_shot_model()

        return self._cached_camera_model

    def get_image_height(self) -> int:
        """Return the height of the image associated with the detection."""
        return self.height

    def get_image_width(self) -> int:
        """Return the width of the image associated with the detection."""
        return self.width

    def get_ordered_object_points(self) -> List[np.array]:
        """Return the object points of the board ordered as a list of arrays."""
        raise NotImplementedError

    def get_ordered_image_points(self) -> List[np.array]:
        """Return the image points of the board ordered as a list of arrays."""
        raise NotImplementedError

    def get_flattened_object_points(self) -> np.array:
        """Return the object points of the board as a (N, 2) array."""
        raise NotImplementedError

    def get_flattened_image_points(self) -> np.array:
        """Return the image points of the board as a (N, 2) array."""
        raise NotImplementedError

    def get_linear_error_rms(self, camera_model: Optional[CameraModel] = None) -> float:
        """Return RMS error product of the projection of the lines of each row of the detection into the line produced by the first and line point of each row."""
        raise NotImplementedError

    def get_center_2d(self) -> np.array:
        """Return the center of detection in the image."""
        if self._cached_center_2d is not None:
            return self._cached_center_2d

        self._cached_center_2d = self.get_flattened_image_points().mean(axis=0)
        return self._cached_center_2d

    def get_reprojection_errors(self, model: Optional[CameraModel] = None) -> np.array:
        """Return the error of projecting the object points into the image and comparing them with the detections."""
        if model is None:
            model = self._get_cached_model()

        if model == self._cached_camera_model and self._cached_reprojection_errors is not None:
            return self._cached_reprojection_errors

        self._cached_camera_model = model
        self._cached_reprojection_errors = model.get_reprojection_errors(self)

        return self._cached_reprojection_errors

    def get_tilt(self, model: Optional[CameraModel] = None) -> float:
        """Return the angle difference between the detection and the camera. Specifically, the pose of the detection points considers +z pointing towards the camera and the camera itself uses +z pointing towards the scene."""
        if model is None:
            model = self._get_cached_model()

        if model == self._cached_camera_model and self._cached_tilt is not None:
            return self._cached_tilt

        rvec, _ = self.get_pose()
        rotmat, _ = cv2.Rodrigues(rvec)
        rotmat[:2, :] *= -1

        v = rotmat @ np.array([0.0, 0.0, 1.0])

        self._cached_camera_model = model
        self._cached_tilt = (180.0 / np.pi) * np.arccos(v[2])

        return self._cached_tilt

    def get_rotation_angles(self, model: Optional[CameraModel] = None) -> Tuple[float, float]:
        """Return the angle differente between the detection and the camera with respect to the x and y axes of the camera."""
        if model is None:
            model = self._get_cached_model()

        if model == self._cached_camera_model and self._cached_rotation_angles is not None:
            return self._cached_rotation_angles

        rvec, _ = self.get_pose()
        rotmat, _ = cv2.Rodrigues(rvec)
        rotmat[:2, :] *= -1

        v = rotmat @ np.array([0.0, 0.0, 1.0])

        self._cached_camera_model = model
        self._cached_rotation_angles = (180.0 / np.pi) * np.arctan2(v[0], v[2]), (
            180.0 / np.pi
        ) * np.arctan2(v[1], v[2])

        return self._cached_rotation_angles

    def get_pose(self, model: Optional[CameraModel] = None) -> Tuple[np.array, np.array]:
        """Return the pose of the detection in rodrigues tuple formulation. If a model is not provided, whe single-shot version is used, which produced only a rough estimation in most cases, and a complete incorrect one in some."""
        if model is None:
            model = self._get_cached_model()

        if model == self._cached_camera_model and self._cached_pose is not None:
            return self._cached_pose

        self._cached_camera_model = model
        self._cached_pose = model.get_pose(board_detection=self)

        return self._cached_pose

    def get_flattened_3d_points(self, model: Optional[CameraModel] = None) -> np.array:
        """Get the image points reprojected into camera coordinates in the 3d space as a (M, 3) array."""
        if model is None:
            model = self._get_cached_model()

        if model == self._cached_camera_model and self._cached_flattened_3d_points is not None:
            return self._cached_pose

        self._cached_camera_model = model

        rvec, tvec = self.get_pose(model)
        rotmat, _ = cv2.Rodrigues(rvec)
        flattened_object_points = self.get_flattened_object_points()

        flattened_3d_points = flattened_object_points @ np.transpose(rotmat) + tvec.reshape(1, 3)

        self._cached_flattened_3d_points = flattened_3d_points

        return self._cached_flattened_3d_points

    def get_normalized_skew(self):
        """Return the skew of the detection as seen from the camera. Subclasses must implement this method."""
        raise NotImplementedError

    def get_normalized_size(self):
        """Return the size of the detection with respect to the image. Subclasses must implement this method."""
        raise NotImplementedError

    def get_speed(self, last: "BoardDetection") -> float:
        """Return the change of the center of the detection with respect to another detection."""
        current_image_points = self.get_flattened_image_points()
        last_image_points = last.get_flattened_image_points()

        return np.linalg.norm(current_image_points - last_image_points, axis=-1).mean()
