from enum import Enum
from typing import List
from typing import Optional
from typing import Tuple

from PySide2.QtCore import Signal
from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.camera_model import CameraModel
from intrinsic_camera_calibrator.parameter import Parameter
from intrinsic_camera_calibrator.parameter import ParameteredClass
import numpy as np


class CollectionStatus(Enum):
    REJECTED = 0
    REDUNDANT = 1
    ACCEPTED = 2


class CollectedData:
    def __init__(self):

        self.detections: List[BoardDetection] = []

        # self.center_pixels = np.array()
        self.normalized_center_x = None
        self.normalized_center_y = None
        self.normalized_skews = None
        self.normalized_sizes = None

        self.center_3d = None
        self.tilt_x = None
        self.tilt_y = None

    def get_detections(self) -> List[BoardDetection]:
        return self.detections

    def get_distances(
        self,
        detection: BoardDetection,
        camera_model: Optional[CameraModel] = None,
        last_n_samples: Optional[int] = 0,
    ) -> Tuple[float, float, float]:

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

    def add_sample(self, detection: BoardDetection, camera_model: Optional[CameraModel] = None):

        self.detections.append(detection)

        self.pre_compute_stats(camera_model)
        pass

    def pre_compute_stats(self, camera_model: CameraModel):

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
        return len(self.detections)


class DataCollector(ParameteredClass):

    new_training_sample = Signal()
    new_evaluation_sample = Signal()
    new_sample = Signal()

    def __init__(self, cfg: dict = dict(), **kwargs):  # noqa C408
        super().__init__(cfg=cfg, **kwargs)

        self.max_samples = Parameter(int, value=500, min_value=6, max_value=500)
        self.decorrelate_eval_samples = Parameter(int, value=5, min_value=1, max_value=100)
        self.max_allowed_tilt = Parameter(float, value=45.0, min_value=0.0, max_value=90.0)

        self.filter_by_speed = Parameter(bool, value=False, min_value=False, max_value=True)
        self.max_allowed_pixel_speed = Parameter(float, value=10, min_value=0.0, max_value=100)
        self.max_allowed_speed = Parameter(float, value=0.1, min_value=0.0, max_value=1.0)

        self.filter_by_reprojection_error = Parameter(
            bool, value=True, min_value=False, max_value=True
        )
        self.max_allowed_max_reprojection_error = Parameter(
            float, value=0.5, min_value=0.0, max_value=2.0
        )
        self.max_allowed_rms_reprojection_error = Parameter(
            float, value=0.3, min_value=0.0, max_value=2.0
        )

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

        self.filter_by_3d_redundancy = Parameter(bool, value=True, min_value=False, max_value=True)
        self.min_3d_center_difference = Parameter(float, value=0.3, min_value=0.0, max_value=2.0)
        self.min_tilt_difference = Parameter(float, value=10.0, min_value=0.0, max_value=True)

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

    def get_training_data(self) -> CollectedData:
        return self.training_data

    def get_evaluation_data(self) -> CollectedData:
        return self.evaluation_data

    def get_flattened_image_training_points(self):

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

        if len(self.cached_image_evaluation_points) == len(self.evaluation_data):
            return self.cached_image_evaluation_points

        self.cached_image_evaluation_points = np.concatenate(
            [
                detection.get_flattened_image_points()
                for detection in self.evaluation_data.get_detections()
            ]
        )
        return self.cached_image_evaluation_points

    def evaluate_redundancy(
        self,
        normalized_2d_center_x_distance: float,
        normalized_2d_center_y_distance: float,
        normalized_skew_distance: float,
        normalized_2d_size_distance: float,
        center_distance: float,
        tilt_x_distance: float,
        tilt_y_distance: float,
    ) -> bool:

        status = True

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

            status &= np.logical_and.reduce(
                np.logical_or.reduce(
                    np.stack(
                        [
                            normalized_2d_center_x_distance_test,
                            normalized_2d_center_y_distance_test,
                            normalized_skew_distance_test,
                            normalized_2d_size_distance_test,
                        ],
                        axis=-1,
                    ),
                    axis=-1,
                )
            )

        if self.filter_by_3d_redundancy:

            center_distance_test = center_distance > self.min_3d_center_difference.value
            tilt_x_distance_test = tilt_x_distance > self.min_tilt_difference.value
            tilt_y_distance_test = tilt_y_distance > self.min_tilt_difference.value

            status &= np.logical_and.reduce(
                np.logical_or.reduce(
                    np.stack(
                        [center_distance_test, tilt_x_distance_test, tilt_y_distance_test], axis=-1
                    ),
                    axis=-1,
                )
            )

        return status

    def process_detection(
        self, detection: BoardDetection, camera_model: Optional[CameraModel] = None
    ) -> CollectionStatus:

        accepted = True

        if self.filter_by_speed.value:
            speed = 0 if self.last_detection is None else detection.get_speed(self.last_detection)
            self.last_detection = detection

            accepted &= speed < self.max_allowed_speed

        if self.filter_by_reprojection_error:
            reprojection_errors = np.linalg.norm(
                detection.get_reprojection_errors(camera_model), axis=-1
            )
            reprojection_error_max = np.abs(reprojection_errors).max()
            reprojection_error_rms = np.sqrt(np.power(reprojection_errors, 2).mean())

            accepted &= (
                reprojection_error_max < self.max_allowed_max_reprojection_error.value
                and reprojection_error_rms < self.max_allowed_rms_reprojection_error.value
            )

        if not accepted:
            return CollectionStatus.REJECTED

        training_detections_distances = self.training_data.get_distances(detection)

        last_n_training_detections_distances = self.training_data.get_distances(
            detection, last_n_samples=self.decorrelate_eval_samples.value
        )
        evaluation_detections_distances = self.evaluation_data.get_distances(detection)

        status_training = self.evaluate_redundancy(*training_detections_distances)
        status_evaluation = self.evaluate_redundancy(
            *last_n_training_detections_distances
        ) and self.evaluate_redundancy(*evaluation_detections_distances)

        if self.heatmap_cells.value != self.training_heatmap.shape[0]:
            self.recompute_heatmaps()

        if status_training and len(self.training_data) < self.max_samples.value:
            self.training_data.add_sample(detection)
            self.training_occupancy_rate = self.update_collection_heatmap(
                self.training_heatmap, detection
            )
            # self.new_training_sample.emit()

        if (
            status_evaluation
            and not status_training
            and len(self.evaluation_data) < self.max_samples.value
        ):
            self.evaluation_data.add_sample(detection)
            self.evaluation_occupancy_rate = self.update_collection_heatmap(
                self.evaluation_heatmap, detection
            )
            # self.new_evaluation_sample.emit()

        if status_training or status_evaluation:
            # self.new_sample.emit()
            return CollectionStatus.ACCEPTED

        return CollectionStatus.REDUNDANT

    def get_num_training_samples(self):
        return len(self.training_data)

    def get_num_evaluation_samples(self):
        return len(self.evaluation_data)

    def recompute_heatmaps(self):

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

        if self.heatmap_cells.value != heatmap.shape[0]:
            self.recompute_heatmaps()

        for point in detection.get_flattened_image_points():
            x = int(heatmap.shape[1] * point[0] / detection.width)
            y = int(heatmap.shape[0] * point[1] / detection.height)
            heatmap[y, x] += 1

        occupied = float(np.count_nonzero(heatmap > 0)) / np.prod(heatmap.shape)
        return occupied

    def get_training_occupancy_rate(self) -> float:
        return self.training_occupancy_rate

    def get_evaluation_occupancy_rate(self) -> float:
        return self.evaluation_occupancy_rate

    def get_training_occupancy_heatmap(self) -> np.array:
        return self.training_heatmap

    def get_evaluation_occupancy_heatmap(self) -> np.array:
        return self.evaluation_heatmap
