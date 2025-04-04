#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

import cv2
from intrinsic_camera_calibrator.types import RectifyMode
import numpy as np


class CameraModelEnum(Enum):
    OPENCV = {"name": "opencv", "display": "OpenCV"}
    CERES = {"name": "ceres", "display": "Ceres"}

    @classmethod
    def from_name(cls, name: str):
        """Return the enum member that matches the name."""
        for model in cls:
            if model.value["name"] == name:
                return model
        raise ValueError(f"{name} not found in {cls.__name__}")

    @classmethod
    def from_index(cls, i: int):
        """Return the enum member by index."""
        return list(cls)[i]

    def get_id(self) -> int:
        """Return the index of the current enum member."""
        return list(self.__class__).index(self)


class CameraModel:
    """Base class of camera model."""

    def __init__(
        self,
        k: Optional[np.array] = None,
        d: Optional[np.array] = None,
        height: Optional[int] = None,
        width: Optional[int] = None,
    ):
        self.k = np.zeros((3, 3)) if k is None else k
        self.d = np.zeros((5,)) if d is None else d
        self.height = height
        self.width = width

        self._cached_undistorted_model = None
        self._cached_undistortion_alpha = np.nan

    def __eq__(self, other: "CameraModel") -> bool:
        """Overload equality operator."""
        return (
            self.height == other.height
            and self.width == other.width
            and (self.k == other.k).all()
            and (self.d == other.d).all()
            and isinstance(self, type(other))
        )

    def calibrate(
        self,
        height: int,
        width: int,
        object_points_list: List[np.array],
        image_points_list: List[np.array],
    ):
        """Calibrate the model."""
        assert len(object_points_list) == len(image_points_list)
        self.height = height
        self.width = width
        self._calibrate_impl(object_points_list, image_points_list)

    def get_pose(
        self,
        board_detection: Optional["BoardDetection"] = None,  # noqa F821
        object_points: Optional[np.array] = None,
        image_points: Optional[np.array] = None,
    ) -> Tuple[np.array, np.array]:
        """Compute the pose of a detection through the PnP algorithm."""
        if board_detection is not None and object_points is None and image_points is None:
            object_points = board_detection.get_flattened_object_points()
            image_points = board_detection.get_flattened_image_points()

        _, rvec, tvec = cv2.solvePnP(object_points, image_points, self.k, self.d)

        return rvec, tvec

    def get_reprojection_rms_error(
        self,
        board_detection: Optional["BoardDetection"] = None,  # noqa F821
        object_points: Optional[np.array] = None,
        image_points: Optional[np.array] = None,
        rvec: Optional[np.array] = None,
        tvec: Optional[np.array] = None,
    ) -> float:
        """Compute the RMS reprojection error of a detection."""
        return np.sqrt(
            np.power(
                self.get_reprojection_errors(
                    board_detection, object_points, image_points, rvec, tvec
                ),
                2,
            ).mean()
        )

    def get_reprojection_error(
        self,
        board_detection: Optional["BoardDetection"] = None,  # noqa F821
        object_points: Optional[np.array] = None,
        image_points: Optional[np.array] = None,
        rvec: Optional[np.array] = None,
        tvec: Optional[np.array] = None,
    ) -> float:
        """Compute the average reprojection error of a detection."""
        return np.linalg.norm(
            self.get_reprojection_errors(board_detection, object_points, image_points, rvec, tvec),
            axis=-1,
        ).mean()

    def get_reprojection_errors(
        self,
        board_detection: Optional["BoardDetection"] = None,  # noqa F821
        object_points: Optional[np.array] = None,
        image_points: Optional[np.array] = None,
        rvec: Optional[np.array] = None,
        tvec: Optional[np.array] = None,
    ) -> np.array:
        """Compute the reprojection errors of a detection."""
        if board_detection is not None and object_points is None and image_points is None:
            object_points = board_detection.get_flattened_object_points()
            image_points = board_detection.get_flattened_image_points()

        if rvec is None or tvec is None:
            rvec, tvec = self.get_pose(object_points=object_points, image_points=image_points)

        num_points, dim = object_points.shape
        projected_points, _ = cv2.projectPoints(object_points, rvec, tvec, self.k, self.d)
        projected_points = projected_points.reshape((num_points, 2))
        return projected_points - image_points

    def as_dict(self, alpha: float = 0.0, rectify_option=0) -> Dict:
        undistorted = self.get_undistorted_camera_model(alpha, rectify_option=rectify_option)
        p = np.zeros((3, 4))
        p[0:3, 0:3] = undistorted.k

        d = {}
        d["image_width"] = self.width
        d["image_height"] = self.height
        d["camera_name"] = ""
        d["camera_matrix"] = {
            "rows": 3,
            "cols": 3,
            "data": [round(e.item(), 11) for e in self.k.flatten()],
        }
        distortion_model_used = "plumb_bob"
        if self.d.size > 5:
            distortion_model_used = "rational_polynomial"
        d["distortion_model"] = distortion_model_used
        d["distortion_coefficients"] = {
            "rows": 1,
            "cols": int(self.d.size),
            "data": [round(e.item(), 11) for e in self.d.flatten()],
        }
        d["projection_matrix"] = {
            "rows": 3,
            "cols": 4,
            "data": [round(e.item(), 11) for e in p.flatten()],
        }
        d["rectification_matrix"] = {
            "rows": 3,
            "cols": 3,
            "data": [round(e.item(), 11) for e in np.eye(3).flatten()],
        }

        return d

    def from_dict(self, d):
        self.width = d["image_width"]
        self.height = d["image_height"]
        self.k = (
            np.array(d["camera_matrix"]["data"])
            .reshape(d["camera_matrix"]["rows"], d["camera_matrix"]["cols"])
            .astype(np.float64)
        )
        self.d = (
            np.array(d["distortion_coefficients"]["data"])
            .reshape(d["distortion_coefficients"]["rows"], d["distortion_coefficients"]["cols"])
            .astype(np.float64)
        )

    def update_config(self, **kwargs):
        """Update the camera model configuration."""
        self._update_config_impl(**kwargs)

    def get_undistorted_camera_model(self, alpha: float, rectify_option):
        """Compute the undistorted version of the camera model."""
        if rectify_option == RectifyMode.FIXED_ASPECT_RATIO:

            def get_rectangles(camera_matrix, dist_coeffs, img_size, new_camera_matrix=None):
                N = 101
                # Generate grid points
                pts = np.zeros((1, N * N, 2), dtype=np.float64)
                k = 0
                for y in range(N):
                    for x in range(N):
                        pts[0, k] = [
                            (x * (img_size[0] - 1)) / (N - 1),
                            (y * (img_size[1] - 1)) / (N - 1),
                        ]
                        k += 1

                # Undistort points
                undistorted_pts = cv2.undistortPoints(
                    pts, camera_matrix, dist_coeffs, P=new_camera_matrix
                )
                undistorted_pts = undistorted_pts.reshape(-1, 2)

                # Initialize variables for inscribed and outer rectangle
                iX0, iX1 = -np.inf, np.inf
                iY0, iY1 = -np.inf, np.inf
                oX0, oX1 = np.inf, -np.inf
                oY0, oY1 = np.inf, -np.inf

                # Calculate inscribed and outer rectangle boundaries
                k = 0
                for y in range(N):
                    for x in range(N):
                        p = undistorted_pts[k]
                        oX0 = min(oX0, p[0])
                        oX1 = max(oX1, p[0])
                        oY0 = min(oY0, p[1])
                        oY1 = max(oY1, p[1])

                        if x == 0:
                            iX0 = max(iX0, p[0])
                        if x == N - 1:
                            iX1 = min(iX1, p[0])
                        if y == 0:
                            iY0 = max(iY0, p[1])
                        if y == N - 1:
                            iY1 = min(iY1, p[1])

                        k += 1

                # Create the rectangles for inner and outer bounds
                inner = (iX0, iY0, iX1 - iX0, iY1 - iY0)
                outer = (oX0, oY0, oX1 - oX0, oY1 - oY0)

                return inner, outer

            size = (self.width, self.height)
            (image_width, image_height) = size
            camera_matrix = self.k
            distortion_coefficients = self.d
            force_aspect_ratio = True
            inner, outer = get_rectangles(camera_matrix, distortion_coefficients, size)

            def roi_to_intrinsics(roi):
                fx = (image_width - 1) / roi[2]
                fy = (image_height - 1) / roi[3]
                cx = -fx * roi[0]
                cy = -fy * roi[1]

                new_image_width = image_width
                new_image_height = image_height

                if force_aspect_ratio:
                    # we want to make sure that fx = fy while also making sure all the roi is valid
                    if fx * roi[3] < image_height - 1:
                        fx = fy
                    else:
                        fy = fx

                intrinsics = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
                return intrinsics, (new_image_width, new_image_height)

            new_intrinsics = None

            if alpha == 0:
                new_intrinsics, new_image_size = roi_to_intrinsics(inner)
            elif alpha == 1:
                new_intrinsics, new_image_size = roi_to_intrinsics(outer)
            else:
                inner_intrinsics, new_image_size = roi_to_intrinsics(inner)
                outer_intrinsics, new_image_size = roi_to_intrinsics(outer)
                new_intrinsics = inner_intrinsics * (1.0 - alpha) + outer_intrinsics * alpha

            roi, _ = get_rectangles(
                camera_matrix, distortion_coefficients, size, new_camera_matrix=new_intrinsics
            )
            roi = list(roi)

            new_image_width, new_image_height = new_image_size

            return type(self)(
                k=new_intrinsics,
                d=np.zeros_like(self.d),
                height=new_image_height,
                width=new_image_width,
            )
        elif rectify_option == RectifyMode.OPENCV:
            undistorted_k, _ = cv2.getOptimalNewCameraMatrix(
                self.k, self.d, (self.width, self.height), alpha
            )

            return type(self)(
                k=undistorted_k, d=np.zeros_like(self.d), height=self.height, width=self.width
            )

    def rectify(self, img: np.array, alpha=0.0, rectify_option=0) -> np.array:
        """Rectifies an image using the current camera model. Alpha is a value in the [0,1] range to regulate how the rectified image is cropped. 0 means that all the pixels in the rectified image are valid whereas 1 keeps all the original pixels from the unrectified image into the rectifies one, filling with zeroes the invalid pixels."""
        if np.all(self.d == 0.0):
            return img

        if self._cached_undistorted_model is None or alpha != self._cached_undistortion_alpha:
            self._cached_undistortion_alpha = alpha
            self._cached_undistorted_model = self.get_undistorted_camera_model(
                alpha=alpha, rectify_option=rectify_option
            )
            (
                self._cached_undistortion_map_x,
                self._cached_undistortion_map_y,
            ) = cv2.initUndistortRectifyMap(
                self.k, self.d, None, self._cached_undistorted_model.k, (self.width, self.height), 5
            )

        return cv2.remap(
            img, self._cached_undistortion_map_x, self._cached_undistortion_map_y, cv2.INTER_LINEAR
        )

    def _calibrate_impl(
        self, object_points_list: List[np.array], image_points_list: List[np.array]
    ):
        """Abstract method to calibrate the camera model."""
        raise NotImplementedError

    def _update_config_impl(self, **kwargs):
        """Abstract method to update the camera model configuration."""
        raise NotImplementedError

    def restart_camera_cached_model(self):
        """Restarts the current cached camera model."""
        self._cached_undistorted_model = None


class CameraModelWithBoardDistortion(CameraModel):
    """An slightly improves model that also incorporates the distortion/bending of the calibration board.."""

    def __init__(
        self,
        k: Optional[np.array] = None,
        d: Optional[np.array] = None,
        height: Optional[int] = None,
        width: Optional[int] = None,
        board_distortion: Optional[np.array] = None,
    ):
        pass
