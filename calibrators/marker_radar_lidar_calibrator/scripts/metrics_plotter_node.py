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

import math

import matplotlib.pyplot as plt
import matplotlib.ticker as mticker  # cspell:ignore mticker
import numpy as np
import rclpy
from rclpy.node import Node
from tier4_calibration_msgs.msg import CalibrationMetrics


class MetricsPlotter:
    def __init__(self):
        self.fig = None
        self.subplots = {}
        self.metrics_data = {}
        self.m_to_cm = 100

        # Define plot colors and styles
        self.color_distance_o = "C0o-"  # Circle marker, solid line for distance
        self.color_yaw_o = "C1o-"  # Circle marker, solid line for yaw
        self.color_distance = "C0"
        self.color_yaw = "C1"

    def initialize_figure(self, methods):
        num_rows = len(methods) + 1  # 1 row for distributions
        num_cols = 4

        # Create the figure with the fixed layout
        self.fig, self.axes = plt.subplots(nrows=num_rows, ncols=num_cols, figsize=(16, 12))
        self.fig.canvas.manager.set_window_title("Metrics and Detection Distributions")

        # Flatten axes for easier indexing and assignment
        self.axes = self.axes.reshape(num_rows, num_cols)

        # Assign subplots for methods
        self.subplots = {method: {} for method in methods}
        for idx, method in enumerate(methods):
            self.subplots[method] = {
                "crossval_distance": self.axes[idx, 0],
                "crossval_yaw": self.axes[idx, 1],
                "average_distance": self.axes[idx, 2],
                "average_yaw": self.axes[idx, 3],
            }

        # Assign subplots for detection distributions
        self.detection_subplots = {
            "range": self.axes[len(methods), 0],
            "pitch": self.axes[len(methods), 1],
            "yaw": self.axes[len(methods), 2],
        }

        # Leave the last column empty for a clean layout
        for ax in self.axes[2, 3:]:
            ax.axis("off")

        plt.tight_layout()
        plt.pause(0.1)

    def initialize_metrics(self, methods):
        for method in methods:
            if method not in self.metrics_data:
                self.metrics_data[method] = {
                    "num_of_reflectors_list": [],
                    "calibrated_distance_error_list": [],
                    "calibrated_yaw_error_list": [],
                    "crossval_sample_list": [],
                    "crossval_distance_error_list": [],
                    "crossval_yaw_error_list": [],
                    "std_crossval_distance_error_list": [],
                    "std_crossval_yaw_error_list": [],
                }

    def plot_label_and_set_xy_lim(self):
        if not hasattr(self, "axes"):
            return  # Skip if the axes are not initialized

        for method, subplots in self.subplots.items():
            subplots["crossval_distance"].set_title(f"{method}\nCross-validation error: distance")
            subplots["crossval_distance"].set_xlabel("Number of tracks")
            subplots["crossval_distance"].set_ylabel("Distance error [cm]")

            subplots["crossval_yaw"].set_title(f"{method}\nCross-validation error: yaw")
            subplots["crossval_yaw"].set_xlabel("Number of tracks")
            subplots["crossval_yaw"].set_ylabel("Yaw error [deg]")

            subplots["average_distance"].set_title(f"{method}\nAverage error: distance")
            subplots["average_distance"].set_xlabel("Number of tracks")
            subplots["average_distance"].set_ylabel("Distance error [cm]")

            subplots["average_yaw"].set_title(f"{method}\nAverage error: yaw")
            subplots["average_yaw"].set_xlabel("Number of tracks")
            subplots["average_yaw"].set_ylabel("Yaw error [deg]")

        for ax in self.axes.flat:
            ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))

    def update_metrics(self, msg):
        # Extract methods from the incoming message
        methods_in_msg = [method.method_name for method in msg.method_metrics]

        # Initialize figure and metrics for new methods
        if not self.metrics_data or set(self.metrics_data.keys()) != set(methods_in_msg):
            self.initialize_figure(methods_in_msg)
            self.initialize_metrics(methods_in_msg)

        # Update metrics for each method in the message
        for method in msg.method_metrics:
            method_name = method.method_name
            metrics = self.metrics_data[method_name]
            metrics["num_of_reflectors_list"] = list(range(1, msg.num_of_converged_tracks + 1))
            metrics["calibrated_distance_error_list"] = [
                value * self.m_to_cm for value in method.calibrated_distance_errors
            ]
            metrics["calibrated_yaw_error_list"] = method.calibrated_yaw_errors

            metrics["crossval_sample_list"] = msg.num_of_samples
            metrics["crossval_distance_error_list"] = [
                value * self.m_to_cm for value in method.avg_crossval_calibrated_distance_errors
            ]
            metrics["crossval_yaw_error_list"] = method.avg_crossval_calibrated_yaw_errors
            metrics["std_crossval_distance_error_list"] = [
                value * self.m_to_cm for value in method.std_crossval_calibrated_distance_errors
            ]
            metrics["std_crossval_yaw_error_list"] = method.std_crossval_calibrated_yaw_errors

    def compute_detection_metrics(self, detections):
        epsilon = 1e-6
        ranges = []
        pitches = []
        yaws = []

        for detection in detections:
            range_ = math.sqrt(detection.x**2 + detection.y**2 + detection.z**2)
            range_ = max(range_, epsilon)
            ranges.append(range_)

            pitch = math.degrees(math.asin(max(-1.0, min(1.0, detection.z / range_))))
            pitches.append(pitch)

            yaw = math.degrees(math.atan2(detection.y, detection.x))
            yaws.append(yaw)

        return ranges, pitches, yaws

    def plot_detection_distributions(self, ranges, pitches, yaws):
        # Clear previous plots
        for subplot in self.detection_subplots.values():
            subplot.clear()

        if not ranges or not pitches or not yaws:
            return

        # Define bin intervals
        range_bin_width = 5  # Interval of 5 meters
        pitch_bin_width = 0.2  # Interval of 0.2 degrees
        yaw_bin_width = 10  # Interval of 10 degrees

        # Create discrete bins
        range_bins = np.arange(
            math.floor(min(ranges) / range_bin_width) * range_bin_width,
            math.ceil(max(ranges) / range_bin_width) * range_bin_width + range_bin_width,
            range_bin_width,
        )
        pitch_bins = np.arange(
            math.floor(min(pitches) / pitch_bin_width) * pitch_bin_width,
            math.ceil(max(pitches) / pitch_bin_width) * pitch_bin_width + pitch_bin_width,
            pitch_bin_width,
        )
        yaw_bins = np.arange(
            math.floor(min(yaws) / yaw_bin_width) * yaw_bin_width,
            math.ceil(max(yaws) / yaw_bin_width) * yaw_bin_width + yaw_bin_width,
            yaw_bin_width,
        )

        # Count occurrences in each bin and cast counts to integers
        range_counts = np.histogram(ranges, bins=range_bins)[0].astype(int)
        pitch_counts = np.histogram(pitches, bins=pitch_bins)[0].astype(int)
        yaw_counts = np.histogram(yaws, bins=yaw_bins)[0].astype(int)

        # Plot range distribution as a bar chart
        self.detection_subplots["range"].bar(
            range_bins[:-1], range_counts, color="C2", alpha=0.7, width=range_bin_width * 0.5
        )
        self.detection_subplots["range"].set_title("Range Distribution")
        self.detection_subplots["range"].set_xlabel("Range [m]")
        self.detection_subplots["range"].set_ylabel("Count")
        self.detection_subplots["range"].set_xticks(range_bins)
        self.detection_subplots["range"].yaxis.set_major_locator(mticker.MaxNLocator(integer=True))

        # Plot pitch distribution as a bar chart
        self.detection_subplots["pitch"].bar(
            pitch_bins[:-1], pitch_counts, color="C3", alpha=0.7, width=pitch_bin_width * 0.5
        )
        self.detection_subplots["pitch"].set_title("Pitch Distribution")
        self.detection_subplots["pitch"].set_xlabel("Pitch [deg]")
        self.detection_subplots["pitch"].set_ylabel("Count")
        self.detection_subplots["pitch"].set_xticks(pitch_bins)
        self.detection_subplots["pitch"].yaxis.set_major_locator(mticker.MaxNLocator(integer=True))

        # Plot yaw distribution as a bar chart
        self.detection_subplots["yaw"].bar(
            yaw_bins[:-1], yaw_counts, color="C4", alpha=0.7, width=yaw_bin_width * 0.5
        )
        self.detection_subplots["yaw"].set_title("Yaw Distribution")
        self.detection_subplots["yaw"].set_xlabel("Yaw [deg]")
        self.detection_subplots["yaw"].set_ylabel("Count")
        self.detection_subplots["yaw"].set_xticks(yaw_bins)
        self.detection_subplots["yaw"].yaxis.set_major_locator(mticker.MaxNLocator(integer=True))

        plt.tight_layout()
        plt.pause(0.1)

    def draw_subplots(self):
        for method, subplots in self.subplots.items():
            metrics = self.metrics_data[method]

            # Clear previous plots
            subplots["crossval_distance"].clear()
            subplots["crossval_yaw"].clear()
            subplots["average_distance"].clear()
            subplots["average_yaw"].clear()

            # Filter data where the number of reflectors >= 3
            filtered_indices = [
                i for i, num in enumerate(metrics["num_of_reflectors_list"]) if num >= 3
            ]

            filtered_reflectors = [metrics["num_of_reflectors_list"][i] for i in filtered_indices]
            filtered_distance_errors = [
                metrics["calibrated_distance_error_list"][i] for i in filtered_indices
            ]
            filtered_yaw_errors = [
                metrics["calibrated_yaw_error_list"][i] for i in filtered_indices
            ]

            if filtered_reflectors and filtered_distance_errors:
                subplots["average_distance"].plot(
                    filtered_reflectors,
                    filtered_distance_errors,
                    self.color_distance_o,
                )
                subplots["average_distance"].annotate(
                    f"{filtered_distance_errors[-1]:.2f}",
                    xy=(filtered_reflectors[-1], filtered_distance_errors[-1]),
                    color=self.color_distance,
                )

            if filtered_reflectors and filtered_yaw_errors:
                subplots["average_yaw"].plot(
                    filtered_reflectors,
                    filtered_yaw_errors,
                    self.color_yaw_o,
                )
                subplots["average_yaw"].annotate(
                    f"{filtered_yaw_errors[-1]:.2f}",
                    xy=(filtered_reflectors[-1], filtered_yaw_errors[-1]),
                    color=self.color_yaw,
                )

            if (
                metrics["crossval_sample_list"]
                and metrics["crossval_distance_error_list"]
                and metrics["std_crossval_distance_error_list"]
            ):
                subplots["crossval_distance"].plot(
                    metrics["crossval_sample_list"],
                    metrics["crossval_distance_error_list"],
                    self.color_distance_o,
                )
                subplots["crossval_distance"].annotate(
                    f"{metrics['crossval_distance_error_list'][-1]:.2f}",
                    xy=(
                        metrics["crossval_sample_list"][-1],
                        metrics["crossval_distance_error_list"][-1],
                    ),
                    color=self.color_distance,
                )
                subplots["crossval_distance"].fill_between(
                    metrics["crossval_sample_list"],
                    np.array(metrics["crossval_distance_error_list"])
                    - np.array(metrics["std_crossval_distance_error_list"]),
                    np.array(metrics["crossval_distance_error_list"])
                    + np.array(metrics["std_crossval_distance_error_list"]),
                    color=self.color_distance,
                    alpha=0.3,
                )

            if (
                metrics["crossval_sample_list"]
                and metrics["crossval_yaw_error_list"]
                and metrics["std_crossval_yaw_error_list"]
            ):
                subplots["crossval_yaw"].plot(
                    metrics["crossval_sample_list"],
                    metrics["crossval_yaw_error_list"],
                    self.color_yaw_o,
                )
                subplots["crossval_yaw"].annotate(
                    f"{metrics['crossval_yaw_error_list'][-1]:.2f}",
                    xy=(
                        metrics["crossval_sample_list"][-1],
                        metrics["crossval_yaw_error_list"][-1],
                    ),
                    color=self.color_yaw,
                )
                subplots["crossval_yaw"].fill_between(
                    metrics["crossval_sample_list"],
                    np.array(metrics["crossval_yaw_error_list"])
                    - np.array(metrics["std_crossval_yaw_error_list"]),
                    np.array(metrics["crossval_yaw_error_list"])
                    + np.array(metrics["std_crossval_yaw_error_list"]),
                    color=self.color_yaw,
                    alpha=0.3,
                )

    def draw_with_msg(self, msg):
        methods_in_msg = [method.method_name for method in msg.method_metrics]

        # Initialize missing methods dynamically
        if not self.metrics_data or set(self.metrics_data.keys()) != set(methods_in_msg):
            self.initialize_figure(methods_in_msg)
            self.initialize_metrics(methods_in_msg)

        self.update_metrics(msg)
        self.draw_subplots()
        self.plot_label_and_set_xy_lim()

        # Handle detections
        ranges, pitches, yaws = self.compute_detection_metrics(msg.detections)
        self.plot_detection_distributions(ranges, pitches, yaws)
        plt.tight_layout()
        plt.pause(0.1)


class MetricsPlotterNode(Node):
    def __init__(self):
        super().__init__("plot_metric")
        self.subscription = self.create_subscription(
            CalibrationMetrics, "calibration_metrics", self.listener_callback, 10
        )
        self.metrics_plotter = MetricsPlotter()

    def listener_callback(self, msg):
        self.metrics_plotter.draw_with_msg(msg)


def main(args=None):
    rclpy.init(args=args)
    metrics_plotter_node = MetricsPlotterNode()
    rclpy.spin(metrics_plotter_node)
    metrics_plotter_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
