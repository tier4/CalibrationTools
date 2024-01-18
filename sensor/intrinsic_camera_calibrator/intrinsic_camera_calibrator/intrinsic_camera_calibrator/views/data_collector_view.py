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


import multiprocessing as mp

from PySide2.QtCore import QObject
from PySide2.QtCore import Signal
from intrinsic_camera_calibrator.camera_model import CameraModel
from intrinsic_camera_calibrator.data_collector import CollectedData
from intrinsic_camera_calibrator.data_collector import DataCollector
import matplotlib.pyplot as plt
import numpy as np


def _create_rotation_heatmap(
    collected_data: CollectedData, camera_model: CameraModel, max_angle: float, angle_resolution=10
) -> np.array:
    """Create an occupancy heatmap of the rotation space of a set of detections."""
    rotations = [
        detection.get_rotation_angles(camera_model) for detection in collected_data.get_detections()
    ]

    cells = int(2 * np.ceil(max_angle / angle_resolution))

    heatmap = np.zeros((cells, cells), dtype=np.int32)

    for rotation in rotations:
        i = np.clip((0.5 * cells) * (rotation[0] / max_angle + 1), 0, cells - 1).astype(np.int32)
        j = np.clip((0.5 * cells) * (rotation[1] / max_angle + 1), 0, cells - 1).astype(np.int32)
        heatmap[j, i] += 1.0

    return heatmap


def _get_2d_points(collected_data: CollectedData) -> np.array:
    """Auxiliary method to group all the dataset image points into an (N, 2) array."""
    return np.array(
        [detection.get_flattened_image_points() for detection in collected_data.get_detections()]
    ).reshape(-1, 2)


def _get_3d_points(collected_data: CollectedData, camera_model: CameraModel) -> np.array:
    """Auxiliary method to group all the dataset object points into an (N, 3) array."""
    return np.array(
        [
            detection.get_flattened_3d_points(camera_model)
            for detection in collected_data.get_detections()
        ]
    ).reshape(-1, 3)


def _plot_data_collection(data_collector: DataCollector, camera_model: CameraModel):
    """Plot the statistics of a dataset. Since matplotlib is used, this function needs to be called into its own process."""
    if (
        len(data_collector.get_training_data().get_detections()) == 0
        and len(data_collector.get_evaluation_data().get_detections()) == 0
    ):
        return

    angle_resolution = data_collector.rotation_heatmap_angle_res.value
    max_angle = data_collector.max_allowed_tilt.value

    fig, axes = plt.subplots(2, 4, figsize=(16, 8))

    if len(data_collector.get_training_data().get_detections()) > 0:
        training_rotation_heatmap = _create_rotation_heatmap(
            data_collector.get_training_data(),
            camera_model,
            max_angle=max_angle,
            angle_resolution=angle_resolution,
        ).astype(np.float32)
        training_rotation_heatmap[training_rotation_heatmap == 0] = np.nan

        training_points_2d = _get_2d_points(data_collector.get_training_data())
        training_points_3d = _get_3d_points(data_collector.get_training_data(), camera_model)

        axes[0, 0].set_title("Training rotation heatmap")

        heatmap = axes[0, 0].imshow(
            training_rotation_heatmap,
            cmap="jet",
            interpolation="nearest",
            extent=[-max_angle, max_angle, -max_angle, max_angle],
        )
        plt.colorbar(heatmap, ax=axes[0, 0])

        h = data_collector.get_training_data().get_detections()[0].get_image_height()
        w = data_collector.get_training_data().get_detections()[0].get_image_width()

        # cSpell:enableCompoundWords
        axes[0, 1].set_title("Training points 2d(x) histogram")
        axes[0, 1].hist(
            training_points_2d[:, 0],
            data_collector.point_2d_hist_bins.value,
            range=[0, w],
            edgecolor="black",
            linewidth=0.5,
        )

        axes[0, 2].set_title("Training points 2d(y) histogram")
        axes[0, 2].hist(
            training_points_2d[:, 1],
            data_collector.point_2d_hist_bins.value,
            range=[0, h],
            edgecolor="black",
            linewidth=0.5,
        )

        axes[0, 3].set_title("Training points 2d(z) histogram")
        axes[0, 3].hist(
            training_points_3d[:, 2],
            data_collector.point_3d_hist_bins.value,
            edgecolor="black",
            linewidth=0.5,
        )

    if len(data_collector.get_evaluation_data().get_detections()) > 0:
        evaluation_rotation_heatmap = _create_rotation_heatmap(
            data_collector.get_evaluation_data(),
            camera_model,
            max_angle=max_angle,
            angle_resolution=angle_resolution,
        ).astype(np.float32)
        evaluation_rotation_heatmap[evaluation_rotation_heatmap == 0] = np.nan

        evaluation_points_2d = _get_2d_points(data_collector.get_evaluation_data())
        evaluation_points_3d = _get_3d_points(data_collector.get_evaluation_data(), camera_model)

        axes[1, 0].set_title("Evaluation rotation heatmap")

        heatmap = axes[1, 0].imshow(
            evaluation_rotation_heatmap,
            cmap="jet",
            interpolation="nearest",
            extent=[-max_angle, max_angle, -max_angle, max_angle],
        )
        plt.colorbar(heatmap, ax=axes[1, 0])

        h = data_collector.get_evaluation_data().get_detections()[0].get_image_height()
        w = data_collector.get_evaluation_data().get_detections()[0].get_image_width()

        axes[1, 1].set_title("Evaluation points 2d(x) histogram")
        axes[1, 1].hist(
            evaluation_points_2d[:, 0],
            data_collector.point_2d_hist_bins.value,
            range=[0, w],
            edgecolor="black",
            linewidth=0.5,
        )

        axes[1, 2].set_title("Evaluation points 2d(y) histogram")
        axes[1, 2].hist(
            evaluation_points_2d[:, 1],
            data_collector.point_2d_hist_bins.value,
            range=[0, h],
            edgecolor="black",
            linewidth=0.5,
        )

        axes[1, 3].set_title("Evaluation points 3d(z) histogram")
        axes[1, 3].hist(
            evaluation_points_3d[:, 2],
            data_collector.point_3d_hist_bins.value,
            edgecolor="black",
            linewidth=0.5,
        )

    plt.show()


class DataCollectorView(QObject):
    failed = Signal()
    closed = Signal()
    plot_request = Signal()

    def __init__(self, data_collector: DataCollector, camera_model: CameraModel):
        super().__init__()
        self.data_collector = data_collector
        self.camera_model = camera_model

    def plot(self):
        self.plot_process = mp.Process(
            target=_plot_data_collection, args=(self.data_collector, self.camera_model), daemon=True
        )
        self.plot_process.start()

    def closeEvent(self, event):
        self.closed.emit()
        event.accept()
        self.deleteLater()
