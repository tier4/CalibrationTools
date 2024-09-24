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

import copy
from typing import List
from typing import Optional
from typing import Tuple

from PySide2.QtCore import Signal
from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.camera_model import CameraModel
from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.parameter import ParameterizedClass
from intrinsic_camera_calibrator.types import CollectionStatus
from intrinsic_camera_calibrator.types import OperationMode
import numpy as np


class CollectedData:
    """A class that contains a database of images and detections. Additionally, it contains a tensorized version of the statistics of the database to accelerate data comparison."""

    def __init__(self):
        self.detections: List[BoardDetection] = []
        self.distorted_images: List[np.array] = []

        self.normalized_center_x = None
        self.normalized_center_y = None
        self.normalized_skews = None
        self.normalized_sizes = None

        self.center_3d = None
        self.tilt_x = None
        self.tilt_y = None

    def clone_without_images(self):
        """Return a lightweight-deep copy of the database (without the images)."""
        clone = copy.deepcopy(self, {id(self.distorted_images): None})
        return clone

    def get_detections(self) -> List[BoardDetection]:
        """Return the detections of the database."""
        return self.detections

    def get_detection(self, index) -> BoardDetection:
        """Return a single detection of the database."""
        return self.detections[index]

    def get_images(self) -> List[np.array]:
        """Return the images from the dataset."""
        return self.distorted_images

    def get_image(self, index) -> np.array:
        """Return a single image from the dataset."""
        return self.distorted_images[index]

    def get_distances(
        self,
        detection: BoardDetection,
        camera_model: Optional[CameraModel] = None,
        last_n_samples: Optional[int] = 0,
    ) -> Tuple[float, float, float]:
        """Compute the 'distance' from a single detection to the ones in the dataset."""
        if len(self.detections) == 0:
            return np.inf, np.inf, np.inf, np.inf, np.inf, np.inf, np.inf

        center_2d = detection.get_center_2d()
        normalized_center_2d_x = center_2d[0] / (
            detection.get_image_width()
            - (
                detection.get_flattened_image_points()[:, 0].max()
                - detection.get_flattened_image_points()[:, 0].min()
            )
        )
        normalized_center_2d_y = center_2d[1] / (
            detection.get_image_height()
            - (
                detection.get_flattened_image_points()[:, 1].max()
                - detection.get_flattened_image_points()[:, 1].min()
            )
        )

        normalized_center_x_distances = np.abs(self.normalized_center_x - normalized_center_2d_x)
        normalized_center_y_distances = np.abs(self.normalized_center_y - normalized_center_2d_y)
        normalized_skews_distances = np.abs(self.normalized_skews - detection.get_normalized_skew())
        normalized_size_distances = np.abs(self.normalized_sizes - detection.get_normalized_size())

        _, pose_tvec = detection.get_pose(camera_model)
        tilt_x, tilt_y = detection.get_rotation_angles(camera_model)

        tilt_x_distance = np.abs(self.tilt_x - tilt_x)
        tilt_y_distance = np.abs(self.tilt_y - tilt_y)
        center_3d_distances = np.linalg.norm(self.center_3d - pose_tvec.reshape((3,)), axis=-1)

        if last_n_samples != 0:
            last_n_samples = min(last_n_samples, len(self.detections))

            normalized_center_x_distances = normalized_center_x_distances[-last_n_samples:]
            normalized_center_y_distances = normalized_center_y_distances[-last_n_samples:]
            normalized_skews_distances = normalized_skews_distances[-last_n_samples:]
            normalized_size_distances = normalized_size_distances[-last_n_samples:]
            tilt_x_distance = tilt_x_distance[-last_n_samples:]
            tilt_y_distance = tilt_y_distance[-last_n_samples:]
            center_3d_distances = center_3d_distances[-last_n_samples:]

        return (
            normalized_center_x_distances,
            normalized_center_y_distances,
            normalized_skews_distances,
            normalized_size_distances,
            tilt_x_distance,
            tilt_y_distance,
            center_3d_distances,
        )

    def add_sample(
        self, image: np.array, detection: BoardDetection, camera_model: Optional[CameraModel] = None
    ):
        """Add a sample to the database and recomputes the statistics."""
        self.distorted_images.append(image)
        self.detections.append(detection)

        self.pre_compute_stats(camera_model)

    def pre_compute_stats(self, camera_model: CameraModel):
        """Compute a tensorized version of the statistics of the database. Needs to be called whenever a sample is added to the database."""
        self.normalized_center_x = np.array(
            [
                detection.get_center_2d()[0]
                / (
                    detection.get_image_width()
                    - (
                        detection.get_flattened_image_points()[:, 0].max()
                        - detection.get_flattened_image_points()[:, 0].min()
                    )
                )
                for detection in self.detections
            ]
        ).flatten()

        self.normalized_center_y = np.array(
            [
                detection.get_center_2d()[1]
                / (
                    detection.get_image_height()
                    - (
                        detection.get_flattened_image_points()[:, 1].max()
                        - detection.get_flattened_image_points()[:, 1].min()
                    )
                )
                for detection in self.detections
            ]
        ).flatten()
        self.normalized_skews = np.array(
            [detection.get_normalized_skew() for detection in self.detections]
        ).flatten()

        self.normalized_sizes = np.array(
            [detection.get_normalized_size() for detection in self.detections]
        ).flatten()

        self.center_3d = np.array(
            [detection.get_pose(camera_model)[1].flatten() for detection in self.detections]
        ).reshape(-1, 3)
        self.tilt_x = np.array(
            [detection.get_rotation_angles(camera_model)[0] for detection in self.detections]
        ).flatten()
        self.tilt_y = np.array(
            [detection.get_rotation_angles(camera_model)[1] for detection in self.detections]
        ).flatten()

    def __len__(self):
        """Overload operator to return the number of samples in the database."""
        return len(self.detections)


class DataCollector(ParameterizedClass):
    """
    Class that manages training and evaluation datasets.

    It also provides statistics about the datasets and implements logic to add new samples with redundancy concerns.
    """

    new_training_sample = Signal()
    new_evaluation_sample = Signal()
    new_sample = Signal()

    def __init__(self, cfg: dict = dict(), **kwargs):  # noqa C408
        super().__init__(cfg=cfg, **kwargs)

        # Option to limit the amount of collection samples in case there are resource constraints
        self.max_samples = Parameter(int, value=500, min_value=6, max_value=5000)

        # In addition to the training dataset we also
        self.decorrelate_eval_samples = Parameter(int, value=5, min_value=1, max_value=100)

        # While strong out-of-plane rotations are needed for good calibrations, they also make detections inaccurate so a trade-off is required
        self.max_allowed_tilt = Parameter(float, value=45.0, min_value=0.0, max_value=90.0)

        # Option to filter out moving targets.
        # This is specially important when using slower shutter speeds
        self.filter_by_speed = Parameter(bool, value=False, min_value=False, max_value=True)
        self.max_allowed_pixel_speed = Parameter(float, value=10, min_value=0.0, max_value=100)
        self.max_allowed_speed = Parameter(float, value=0.1, min_value=0.0, max_value=1.0)

        # One way to filter out bad detections is too perform camera-calibration with one single sample.
        # If the model can not fit the sample, it should be considered an outlier
        self.filter_by_reprojection_error = Parameter(
            bool, value=True, min_value=False, max_value=True
        )
        self.max_allowed_max_reprojection_error = Parameter(
            float, value=0.5, min_value=0.0, max_value=2.0
        )
        self.max_allowed_rms_reprojection_error = Parameter(
            float, value=0.3, min_value=0.0, max_value=2.0
        )

        # New samples are required to be "different" from the ones already in the dataset
        # Some criteria include the center of the detection, the area of the detection (related to the distance from the camera) and the estimated out-of-plane rotation in the form of skew
        self.filter_by_2d_redundancy = Parameter(bool, value=True, min_value=False, max_value=True)
        self.min_normalized_2d_center_difference = Parameter(
            float, value=0.05, min_value=0.0, max_value=1.0
        )
        self.min_normalized_skew_difference = Parameter(
            float, value=0.05, min_value=0.0, max_value=1.0
        )
        self.min_normalized_2d_size_difference = Parameter(
            float, value=0.05, min_value=0.0, max_value=1.0
        )

        # Other criteria for new samples is using 3d statistics
        # They have the advantage of considering the different out-of-plane rotations instead of a single scalar (e.g., differentiation of left and right rotations)
        self.filter_by_3d_redundancy = Parameter(bool, value=True, min_value=False, max_value=True)
        self.min_3d_center_difference = Parameter(float, value=1.0, min_value=0.1, max_value=100.0)
        self.min_tilt_difference = Parameter(float, value=15.0, min_value=0.0, max_value=90)

        # Visualization parameters
        self.heatmap_cells = Parameter(int, value=16, min_value=2, max_value=100)
        self.rotation_heatmap_angle_res = Parameter(int, value=10, min_value=5, max_value=20)
        self.point_2d_hist_bins = Parameter(int, value=20, min_value=2, max_value=100)
        self.point_3d_hist_bins = Parameter(int, value=20, min_value=2, max_value=100)

        self.set_parameters(**cfg)

        self.training_data = CollectedData()
        self.evaluation_data = CollectedData()

        self.last_detection = None
        self.cached_image_training_points = np.array([])
        self.cached_image_evaluation_points = np.array([])

        self.training_heatmap = np.zeros((self.heatmap_cells.value, self.heatmap_cells.value))
        self.evaluation_heatmap = np.zeros((self.heatmap_cells.value, self.heatmap_cells.value))
        self.training_occupancy_rate = 0.0
        self.evaluation_occupancy_rate = 0.0

    def clone_without_images(self):
        """Return a lightweight-deep copy of the databases (without the images)."""
        training_data = self.training_data.clone_without_images()
        evaluation_data = self.evaluation_data.clone_without_images()
        clone = copy.deepcopy(
            self, {id(self.training_data): training_data, id(self.evaluation_data): evaluation_data}
        )

        return clone

    def get_training_data(self) -> CollectedData:
        """Return the training data."""
        return self.training_data

    def get_evaluation_data(self) -> CollectedData:
        """Return the evaluation data."""
        return self.evaluation_data

    def get_training_detections(self) -> List[BoardDetection]:
        """Return the training detections."""
        return self.training_data.get_detections()

    def get_evaluation_detections(self) -> List[BoardDetection]:
        """Return the evaluation detections."""
        return self.evaluation_data.get_detections()

    def get_training_images(self) -> List[np.array]:
        """Return the training images."""
        return self.training_data.get_images()

    def get_evaluation_images(self) -> List[np.array]:
        """Return the evaluation images."""
        return self.evaluation_data.get_images()

    def get_training_detection(self, index) -> BoardDetection:
        """Return the a single training detection."""
        return self.training_data.get_detection(index)

    def get_evaluation_detection(self, index) -> BoardDetection:
        """Return the a single evaluation detection."""
        return self.evaluation_data.get_detection(index)

    def get_training_image(self, index) -> np.array:
        """Return the a single training image."""
        return self.training_data.get_image(index)

    def get_evaluation_image(self, index) -> np.array:
        """Return the a single evaluation image."""
        return self.evaluation_data.get_image(index)

    def get_flattened_image_training_points(self):
        """Return a flattened (N,2) array of the training image points."""
        if len(self.cached_image_training_points) == len(self.training_data):
            return self.cached_image_training_points

        self.cached_image_training_points = np.concatenate(
            [
                detection.get_flattened_image_points()
                for detection in self.training_data.get_detections()
            ]
        )
        return self.cached_image_training_points

    def get_flattened_image_evaluation_points(self):
        """Return a flattened (N,2) array of the evaluation image points."""
        if len(self.cached_image_evaluation_points) == len(self.evaluation_data):
            return self.cached_image_evaluation_points

        self.cached_image_evaluation_points = np.concatenate(
            [
                detection.get_flattened_image_points()
                for detection in self.evaluation_data.get_detections()
            ]
        )
        return self.cached_image_evaluation_points

    def get_num_training_samples(self):
        """Return the number of training samples."""
        return len(self.training_data)

    def get_num_evaluation_samples(self):
        """Return the number of evaluation samples."""
        return len(self.evaluation_data)

    def get_training_occupancy_rate(self) -> float:
        """Return the training occupancy rate, which is defined as the ratio of cells in the pixel space that have at least one image point."""
        return self.training_occupancy_rate

    def get_evaluation_occupancy_rate(self) -> float:
        """Return the evaluation occupancy rate, which is defined as the ratio of cells in the pixel space that have at least one image point."""
        return self.evaluation_occupancy_rate

    def get_training_occupancy_heatmap(self) -> np.array:
        """Return the training heatmap, which defines the parts of the pixel space that have image points in it."""
        return self.training_heatmap

    def get_evaluation_occupancy_heatmap(self) -> np.array:
        """Return the evaluation heatmap, which defines the parts of the pixel space that have image points in it."""
        return self.evaluation_heatmap

    def recompute_heatmaps(self):
        """Recomputes the heatmaps of training and evaluation databases."""
        self.training_heatmap = np.zeros((self.heatmap_cells.value, self.heatmap_cells.value))
        self.evaluation_heatmap = np.zeros((self.heatmap_cells.value, self.heatmap_cells.value))

        [
            self.update_collection_heatmap(self.training_heatmap, detection)
            for detection in self.training_data.get_detections()
        ]
        [
            self.update_collection_heatmap(self.evaluation_heatmap, detection)
            for detection in self.evaluation_data.get_detections()
        ]

        self.training_occupancy_rate = float(np.count_nonzero(self.training_heatmap > 0)) / np.prod(
            self.training_heatmap.shape
        )
        self.evaluation_occupancy_rate = float(
            np.count_nonzero(self.evaluation_heatmap > 0)
        ) / np.prod(self.evaluation_heatmap.shape)

    def update_collection_heatmap(self, heatmap: np.array, detection: BoardDetection) -> float:
        """Update a heatmap with a single detection's image points."""
        if self.heatmap_cells.value != heatmap.shape[0]:
            self.recompute_heatmaps()

        for point in detection.get_flattened_image_points():
            x = int(heatmap.shape[1] * point[0] / detection.width)
            y = int(heatmap.shape[0] * point[1] / detection.height)
            heatmap[y, x] += 1

        occupied = float(np.count_nonzero(heatmap > 0)) / np.prod(heatmap.shape)
        return occupied

    def evaluate_redundancy(
        self,
        normalized_2d_center_x_distance: float,
        normalized_2d_center_y_distance: float,
        normalized_skew_distance: float,
        normalized_2d_size_distance: float,
        tilt_x_distance: float,
        tilt_y_distance: float,
        center_distance: float,
    ) -> bool:
        """Evaluate if the distances from a detection to the dataset merits adding it."""
        status = not self.filter_by_2d_redundancy.value and not self.filter_by_3d_redundancy.value

        if self.filter_by_2d_redundancy.value:
            normalized_2d_center_x_distance_test = (
                normalized_2d_center_x_distance > self.min_normalized_2d_center_difference.value
            )
            normalized_2d_center_y_distance_test = (
                normalized_2d_center_y_distance > self.min_normalized_2d_center_difference.value
            )
            normalized_skew_distance_test = (
                normalized_skew_distance > self.min_normalized_skew_difference.value
            )
            normalized_2d_size_distance_test = (
                normalized_2d_size_distance > self.min_normalized_2d_size_difference.value
            )

            joint_test = np.stack(
                [
                    normalized_2d_center_x_distance_test,
                    normalized_2d_center_y_distance_test,
                    normalized_skew_distance_test,
                    normalized_2d_size_distance_test,
                ],
                axis=-1,
            ).sum(axis=-1)

            status_2d = joint_test.min() > 0
            status |= status_2d

        if self.filter_by_3d_redundancy.value:
            center_distance_test = center_distance > self.min_3d_center_difference.value
            tilt_x_distance_test = tilt_x_distance > self.min_tilt_difference.value
            tilt_y_distance_test = tilt_y_distance > self.min_tilt_difference.value

            joint_test = np.stack(
                [center_distance_test, tilt_x_distance_test, tilt_y_distance_test], axis=-1
            ).sum(axis=-1)
            status_3d = joint_test.min() > 0
            status |= status_3d

        return status

    def process_detection(
        self,
        image: np.array,
        detection: BoardDetection,
        camera_model: Optional[CameraModel] = None,
        mode: OperationMode = OperationMode.CALIBRATION,
    ) -> CollectionStatus:
        """Evaluate if a detection should be added to either the training or evaluation dataset."""
        accepted = True

        if self.filter_by_speed.value:
            speed = 0 if self.last_detection is None else detection.get_speed(self.last_detection)
            self.last_detection = detection

            accepted &= speed < self.max_allowed_speed

        if self.filter_by_reprojection_error:
            reprojection_errors = detection.get_reprojection_errors()
            reprojection_errors_norm = np.linalg.norm(reprojection_errors, axis=-1)

            reprojection_error_max = reprojection_errors_norm.max()
            reprojection_error_rms = np.sqrt(np.power(reprojection_errors, 2).mean())

            accepted &= (
                reprojection_error_max < self.max_allowed_max_reprojection_error.value
                and reprojection_error_rms < self.max_allowed_rms_reprojection_error.value
            )

        if not accepted:
            return CollectionStatus.REJECTED

        training_detections_distances = self.training_data.get_distances(
            detection, camera_model=camera_model
        )

        # cSpell:enableCompoundWords
        last_n_training_detections_distances = self.training_data.get_distances(
            detection, camera_model=camera_model, last_n_samples=self.decorrelate_eval_samples.value
        )
        evaluation_detections_distances = self.evaluation_data.get_distances(
            detection, camera_model=camera_model
        )

        status_training = mode == OperationMode.CALIBRATION and self.evaluate_redundancy(
            *training_detections_distances
        )
        status_evaluation = self.evaluate_redundancy(
            *last_n_training_detections_distances
        ) and self.evaluate_redundancy(*evaluation_detections_distances)

        if self.heatmap_cells.value != self.training_heatmap.shape[0]:
            self.recompute_heatmaps()

        if status_training and len(self.training_data) < self.max_samples.value:
            self.training_data.add_sample(
                image=image, detection=detection, camera_model=camera_model
            )
            self.training_occupancy_rate = self.update_collection_heatmap(
                self.training_heatmap, detection
            )

        if (
            status_evaluation
            and not status_training
            and len(self.evaluation_data) < self.max_samples.value
        ):
            self.evaluation_data.add_sample(
                image=image, detection=detection, camera_model=camera_model
            )
            self.evaluation_occupancy_rate = self.update_collection_heatmap(
                self.evaluation_heatmap, detection
            )

        if status_training or status_evaluation:
            return CollectionStatus.ACCEPTED

        return CollectionStatus.REDUNDANT
