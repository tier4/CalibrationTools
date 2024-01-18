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

from intrinsic_camera_calibrator.board_detections.board_detection import BoardDetection
from intrinsic_camera_calibrator.camera_model import CameraModel
import matplotlib.pyplot as plt
import numpy as np


def get_entropy(array: np.array):
    """Compute the entropy of an array interpreting it as flattened 1D discrete distribution."""
    array = array.flatten()
    array = array[array > 0] / array.sum()
    return -(array * np.log2(array)).sum()


def add_detection(
    detection: BoardDetection,
    camera_model: CameraModel,
    pixel_occupancy: np.array,
    tilt_occupancy: np.array,
    pixel_cells: int,
    tilt_cells: int,
    max_tilt_deg: float,
):
    """Add a detection to occupancy heatmaps."""
    tilt = detection.get_rotation_angles(camera_model)
    tilt_x = tilt[0] / max_tilt_deg
    tilt_y = tilt[1] / max_tilt_deg

    if np.abs(tilt_x) > 1.0 or np.abs(tilt_y) > 1.0:
        return

    tilt_i = int(np.clip((0.5 * tilt_cells) * (tilt_x + 1), 0, tilt_cells - 1))
    tilt_j = int(np.clip((0.5 * tilt_cells) * (tilt_y + 1), 0, tilt_cells - 1))
    tilt_occupancy[tilt_j, tilt_i] += 1.0

    for point in detection.get_flattened_image_points():
        pixel_i = int(pixel_cells * point[0] / detection.width)
        pixel_j = int(pixel_cells * point[1] / detection.height)
        pixel_occupancy[pixel_j, pixel_i] += 1


def add_detection_errors(
    detection: BoardDetection,
    camera_model: CameraModel,
    pixel_errors: List[List[List[float]]],
    tilt_errors: np.array,
    pixel_cells: int,
    tilt_cells: int,
    max_tilt_deg: float,
):
    """Add a detection to error heatmaps."""
    tilt = detection.get_rotation_angles(camera_model)
    tilt_x = tilt[0] / max_tilt_deg
    tilt_y = tilt[1] / max_tilt_deg

    if np.abs(tilt_x) > 1.0 or np.abs(tilt_y) > 1.0:
        return

    tilt_i = int(np.clip((0.5 * tilt_cells) * (tilt_x + 1), 0, tilt_cells - 1))
    tilt_j = int(np.clip((0.5 * tilt_cells) * (tilt_y + 1), 0, tilt_cells - 1))

    errors = detection.get_reprojection_errors()

    tilt_errors[tilt_j][tilt_i].append(np.sqrt(np.power(errors, 2).mean()))

    for point, error in zip(detection.get_flattened_image_points(), errors):
        pixel_i = int(pixel_cells * point[0] / detection.width)
        pixel_j = int(pixel_cells * point[1] / detection.height)
        pixel_errors[pixel_j][pixel_i].append(np.sqrt(np.power(error, 2).mean()))


def plot_calibration_data_statistics(
    calibrated_model: CameraModel,
    training_detections: List[BoardDetection],
    pre_rejection_inlier_detections: List[BoardDetection],
    subsampled_detections: List[BoardDetection],
    post_rejection_inlier_detections: List[BoardDetection],
    evaluation_detections: List[BoardDetection],
    pixel_cells: int,
    tilt_resolution: float,
    max_tilt_deg: float,
    z_cells: int,
):
    tilt_cells: int = int(2 * np.ceil(max_tilt_deg / tilt_resolution))

    fig, axes = plt.subplots(3, 5, figsize=(20, 12))
    fig.canvas.set_window_title("Calibration data statistics")

    def process_detections(detections: List[BoardDetection]):
        pixel_occupancy = np.zeros((pixel_cells, pixel_cells))
        tilt_occupancy = np.zeros((tilt_cells, tilt_cells))

        z_list = []

        for detection in detections:
            add_detection(
                detection,
                calibrated_model,
                pixel_occupancy,
                tilt_occupancy,
                pixel_cells,
                tilt_cells,
                max_tilt_deg,
            )

            rvec, tvec = detection.get_pose(calibrated_model)
            z_list.append(tvec.flatten()[2])

        return pixel_occupancy, tilt_occupancy, np.array(z_list)

    def plot_detection_set(i: int, name: str, detections: List[BoardDetection]):
        if len(detections) == 0:
            return

        pixel_heatmap, rotation_heatmap, z_array = process_detections(detections)
        pixel_entropy = get_entropy(pixel_heatmap)
        rotation_entropy = get_entropy(rotation_heatmap)

        pixel_heatmap[pixel_heatmap == 0] = np.nan
        rotation_heatmap[rotation_heatmap == 0] = np.nan

        height = detections[0].get_image_height()
        width = detections[0].get_image_width()

        axes[0, i].set_title(f"{name} pixel heatmap\n(entropy={pixel_entropy:.2f})")
        pixel_heatmap_ax = axes[0, i].imshow(
            pixel_heatmap,
            cmap="jet",
            interpolation="nearest",
            extent=[0, width, 0, height],
            aspect="equal",
        )
        plt.colorbar(pixel_heatmap_ax, ax=axes[0, i])

        axes[1, i].set_title(f"{name} rotation heatmap\n(entropy{rotation_entropy:.2f})")
        rotation_heatmap_ax = axes[1, i].imshow(
            rotation_heatmap,
            cmap="jet",
            interpolation="nearest",
            extent=[-max_tilt_deg, max_tilt_deg, -max_tilt_deg, max_tilt_deg],
            aspect="equal",
        )
        plt.colorbar(rotation_heatmap_ax, ax=axes[1, i])

        # cSpell:enableCompoundWords
        axes[2, i].set_title(f"{name} z histogram")
        axes[2, i].hist(
            z_array,
            z_cells,
            edgecolor="black",
            linewidth=0.5,
        )

    plot_detection_set(0, "Training", training_detections)
    plot_detection_set(1, "Pre rejection inliers", pre_rejection_inlier_detections)
    plot_detection_set(2, "Subsampled", subsampled_detections)
    plot_detection_set(3, "Post rejection inliers", post_rejection_inlier_detections)
    plot_detection_set(4, "Evaluation", evaluation_detections)
    plt.show()


def plot_calibration_results_statistics(
    calibrated_model: CameraModel,
    training_detections: List[BoardDetection],
    inlier_detections: List[BoardDetection],
    evaluation_detections: List[BoardDetection],
    pixel_cells: int,
    tilt_resolution: float,
    max_tilt_deg: float,
    z_cells: int,
):
    tilt_cells: int = int(2 * np.ceil(max_tilt_deg / tilt_resolution))

    fig1, axes1 = plt.subplots(3, 4, figsize=(20, 12))
    fig2, axes2 = plt.subplots(3, 1, figsize=(20, 12))

    fig1.canvas.set_window_title("Calibration result statistics")
    fig2.canvas.set_window_title("Calibration result statistics vs single shot calibration")

    def process_detections(detections: List[BoardDetection]):
        pixel_errors = [[[] for i in range(pixel_cells)] for j in range(pixel_cells)]
        tilt_errors = [[[] for i in range(tilt_cells)] for j in range(tilt_cells)]

        for detection in detections:
            add_detection_errors(
                detection,
                calibrated_model,
                pixel_errors,
                tilt_errors,
                pixel_cells,
                tilt_cells,
                max_tilt_deg,
            )

        pixel_errors_mean = np.nan * np.ones((pixel_cells, pixel_cells))
        pixel_errors_std = np.nan * np.ones((pixel_cells, pixel_cells))
        tilt_errors_mean = np.nan * np.ones((tilt_cells, tilt_cells))
        tilt_errors_std = np.nan * np.ones((tilt_cells, tilt_cells))

        for j in range(pixel_cells):
            for i in range(pixel_cells):
                if len(pixel_errors[j][i]) > 0:
                    pixel_errors_mean[j, i] = np.array(pixel_errors[j][i]).mean()
                    pixel_errors_std[j, i] = np.array(pixel_errors[j][i]).std()

        for j in range(tilt_cells):
            for i in range(tilt_cells):
                if len(tilt_errors[j][i]) > 0:
                    tilt_errors_mean[j, i] = np.array(tilt_errors[j][i]).mean()
                    tilt_errors_std[j, i] = np.array(tilt_errors[j][i]).std()

        return pixel_errors_mean, pixel_errors_std, tilt_errors_mean, tilt_errors_std

    def plot_detection_set(j: int, name: str, detections: List[BoardDetection]):
        if len(detections) == 0:
            return

        pixel_errors_mean, pixel_errors_std, tilt_errors_mean, tilt_errors_std = process_detections(
            detections
        )

        height = detections[0].get_image_height()
        width = detections[0].get_image_width()

        axes1[j, 0].set_title(f"{name}: rms error (mean)")
        pixel_rms_mean_ax = axes1[j, 0].imshow(
            pixel_errors_mean,
            cmap="jet",
            interpolation="nearest",
            extent=[0, width, 0, height],
            aspect="equal",
        )
        plt.colorbar(pixel_rms_mean_ax, ax=axes1[j, 0])

        axes1[j, 1].set_title(f"{name}: rms error (std)")
        pixel_rms_std_ax = axes1[j, 1].imshow(
            pixel_errors_std,
            cmap="jet",
            interpolation="nearest",
            extent=[0, width, 0, height],
            aspect="equal",
        )
        plt.colorbar(pixel_rms_std_ax, ax=axes1[j, 1])

        axes1[j, 2].set_title(f"{name}: rms error (mean)")
        tilt_rms_mean_ax = axes1[j, 2].imshow(
            tilt_errors_mean,
            cmap="jet",
            interpolation="nearest",
            extent=[-max_tilt_deg, max_tilt_deg, -max_tilt_deg, max_tilt_deg],
            aspect="equal",
        )
        plt.colorbar(tilt_rms_mean_ax, ax=axes1[j, 2])

        axes1[j, 3].set_title(f"{name}: rms error (std)")
        tilt_rms_std_ax = axes1[j, 3].imshow(
            tilt_errors_std,
            cmap="jet",
            interpolation="nearest",
            extent=[-max_tilt_deg, max_tilt_deg, -max_tilt_deg, max_tilt_deg],
            aspect="equal",
        )
        plt.colorbar(tilt_rms_std_ax, ax=axes1[j, 3])

    def plot_calibration_vs_single_shot_calibration(j, name, detections: List[BoardDetection]):
        label = np.array([str(i) for i in range(len(detections))])

        calibrated_errors = np.array(
            [calibrated_model.get_reprojection_rms_error(detection) for detection in detections]
        )
        single_shot_errors = np.array(
            [
                np.sqrt(np.power(detection.get_reprojection_errors(), 2).mean())
                for detection in detections
            ]
        )

        x_axis = np.arange(len(detections))

        sorted_idxs = np.argsort(calibrated_errors)
        calibrated_errors = np.take_along_axis(calibrated_errors, sorted_idxs, axis=0)
        single_shot_errors = np.take_along_axis(single_shot_errors, sorted_idxs, axis=0)
        label = np.take_along_axis(label, sorted_idxs, axis=0)

        axes2[j].set_title(name)
        axes2[j].bar(x_axis - 0.2, calibrated_errors, width=0.4, label="Calibrated intrinsics")
        axes2[j].bar(
            x_axis + 0.2,
            single_shot_errors,
            width=0.4,
            label="Single-shot intrinsics (lower bound)",
        )

        # cSpell:ignore xticks
        axes2[j].set_xticks(x_axis, label, rotation="vertical")
        axes2[j].legend()

    plot_detection_set(0, "Training", training_detections)
    plot_detection_set(1, "Inliers", inlier_detections)
    plot_detection_set(2, "Evaluation", evaluation_detections)

    plot_calibration_vs_single_shot_calibration(0, "Training", training_detections)
    plot_calibration_vs_single_shot_calibration(1, "Inliers", inlier_detections)
    plot_calibration_vs_single_shot_calibration(2, "Evaluation", evaluation_detections)

    plt.show()
