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

import math

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class MetricsPlotter:
    def __init__(self):
        self.fig, self.axes = plt.subplots(nrows=2, ncols=2, figsize=(8, 6))
        self.subplot0 = self.axes[0, 0]
        self.subplot1 = self.axes[0, 1]
        self.subplot2 = self.axes[1, 0]
        self.subplot3 = self.axes[1, 1]
        self.fig.canvas.manager.set_window_title("Metrics plotter")

        self.color_distance_o = "C0o-"
        self.color_yaw_o = "C1o-"
        self.color_distance = "C0"
        self.color_yaw = "C1"

        (
            self.num_of_reflectors_list,
            self.calibration_distance_error_list,
            self.calibration_yaw_error_list,
            self.crossval_sample_list,
            self.crossval_distance_error_list,
            self.crossval_yaw_error_list,
            self.std_crossval_distance_error_list,
            self.std_crossval_yaw_error_list,
        ) = ([], [], [], [], [], [], [], [])

        self.m_to_cm = 100

        self.plot_label_and_set_xy_lim()
        plt.tight_layout()
        plt.pause(0.1)

    def plot_label_and_set_xy_lim(self):
        self.subplot0.set_title("cross-validation error: distance")
        self.subplot0.set_xlabel("number of tracks")
        self.subplot0.set_ylabel("distance error [cm]")

        self.subplot1.set_title("cross-validation error: yaw")
        self.subplot1.set_xlabel("number of tracks")
        self.subplot1.set_ylabel("yaw error [deg]")

        self.subplot2.set_title("average error: distance")
        self.subplot2.set_xlabel("number of tracks")
        self.subplot2.set_ylabel("distance error [cm]")

        self.subplot3.set_title("average error: yaw")
        self.subplot3.set_xlabel("number of tracks")
        self.subplot3.set_ylabel("yaw error [deg]")

        max_ylim0 = (
            max(self.crossval_distance_error_list) if self.crossval_distance_error_list else 5
        )
        max_ylim1 = max(self.crossval_yaw_error_list) if self.crossval_yaw_error_list else 1
        max_ylim2 = (
            max(self.calibration_distance_error_list) if self.calibration_distance_error_list else 5
        )
        max_ylim3 = max(self.calibration_yaw_error_list) if self.calibration_yaw_error_list else 1

        cross_val_xlim = (
            self.crossval_sample_list[-1]
            if self.crossval_sample_list and self.crossval_sample_list[-1] >= 5
            else 5
        )
        avg_xlim = (
            self.num_of_reflectors_list[-1]
            if self.num_of_reflectors_list and self.num_of_reflectors_list[-1] >= 5
            else 5
        )

        self.subplot0.set_xlim(2.9, cross_val_xlim + 0.3)
        self.subplot1.set_xlim(2.9, cross_val_xlim + 0.3)
        self.subplot2.set_xlim(2.9, avg_xlim + 0.3)
        self.subplot3.set_xlim(2.9, avg_xlim + 0.3)

        self.subplot0.set_ylim(0, max_ylim0 + 5)
        self.subplot1.set_ylim(0, max_ylim1 + 0.1)
        self.subplot2.set_ylim(0, max_ylim2 + 5)
        self.subplot3.set_ylim(0, max_ylim3 + 0.1)

        for ax in self.axes.flat:
            ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))

    def is_delete_operation(self, msg_array):
        return self.num_of_reflectors_list and msg_array[0] < self.num_of_reflectors_list[-1]

    def remove_avg_error_from_list(self):
        for i in range(min(2, len(self.num_of_reflectors_list))):
            self.calibration_distance_error_list.pop()
            self.calibration_yaw_error_list.pop()
            self.num_of_reflectors_list.pop()

    def add_avg_error_to_list(self, msg_array):
        num_of_reflectors = msg_array[0]
        calibration_distance_error = msg_array[1] * self.m_to_cm
        calibration_yaw_error = 0 if math.isnan(msg_array[2]) else msg_array[2]

        if num_of_reflectors >= 3:
            self.num_of_reflectors_list.append(num_of_reflectors)
            self.calibration_distance_error_list.append(calibration_distance_error)
            self.calibration_yaw_error_list.append(calibration_yaw_error)

    def add_crossval_error_to_list(self, msg_array):
        (
            self.crossval_sample_list,
            self.crossval_distance_error_list,
            self.crossval_yaw_error_list,
            self.std_crossval_distance_error_list,
            self.std_crossval_yaw_error_list,
        ) = ([], [], [], [], [])

        for i in range((len(msg_array) - 3) // 5):
            self.crossval_sample_list.append(msg_array[3 + i * 5])
            self.crossval_distance_error_list.append(msg_array[3 + i * 5 + 1] * self.m_to_cm)
            self.crossval_yaw_error_list.append(msg_array[3 + i * 5 + 2])
            self.std_crossval_distance_error_list.append(msg_array[3 + i * 5 + 3] * self.m_to_cm)
            self.std_crossval_yaw_error_list.append(msg_array[3 + i * 5 + 4])

    def draw_avg_subplots(self):
        self.subplot2.clear()
        self.subplot3.clear()

        self.subplot2.plot(
            self.num_of_reflectors_list,
            self.calibration_distance_error_list,
            self.color_distance_o,
        )
        self.subplot3.plot(
            self.num_of_reflectors_list,
            self.calibration_yaw_error_list,
            self.color_yaw_o,
        )

        if len(self.num_of_reflectors_list) > 0:
            # draw annotations for the last point
            self.subplot2.annotate(
                f"{self.calibration_distance_error_list[-1]:.2f}",  # noqa E231
                xy=(self.num_of_reflectors_list[-1], self.calibration_distance_error_list[-1]),
                color=self.color_distance,
            )
            self.subplot3.annotate(
                f"{self.calibration_yaw_error_list[-1]:.2f}",  # noqa E231
                xy=(self.num_of_reflectors_list[-1], self.calibration_yaw_error_list[-1]),
                color=self.color_yaw,
            )

    def draw_crossval_subplots(self):
        self.subplot0.clear()
        self.subplot1.clear()

        self.subplot0.plot(
            self.crossval_sample_list,
            self.crossval_distance_error_list,
            self.color_distance_o,
        )
        self.subplot1.plot(
            self.crossval_sample_list,
            self.crossval_yaw_error_list,
            self.color_yaw_o,
        )

        # draw std and mean of error
        self.subplot0.fill_between(
            self.crossval_sample_list,
            np.array(self.crossval_distance_error_list)
            - np.array(self.std_crossval_distance_error_list),
            np.array(self.crossval_distance_error_list)
            + np.array(self.std_crossval_distance_error_list),
            color=self.color_distance,
            alpha=0.3,
        )
        self.subplot1.fill_between(
            self.crossval_sample_list,
            np.array(self.crossval_yaw_error_list) - np.array(self.std_crossval_yaw_error_list),
            np.array(self.crossval_yaw_error_list) + np.array(self.std_crossval_yaw_error_list),
            color=self.color_yaw,
            alpha=0.3,
        )

        # annotate the last value
        if len(self.crossval_sample_list) > 0:
            self.subplot0.annotate(
                f"{self.crossval_distance_error_list[-1]:.2f}",  # noqa E231
                xy=(self.crossval_sample_list[-1], self.crossval_distance_error_list[-1]),
                color=self.color_distance,
            )
            self.subplot1.annotate(
                f"{self.crossval_yaw_error_list[-1]:.2f}",  # noqa E231
                xy=(self.crossval_sample_list[-1], self.crossval_yaw_error_list[-1]),
                color=self.color_yaw,
            )

    def draw_with_msg(self, msg):
        msg_array = msg.data
        if self.is_delete_operation(msg_array):
            self.remove_avg_error_from_list()
        self.add_avg_error_to_list(msg_array)
        self.add_crossval_error_to_list(msg_array)
        self.draw_avg_subplots()
        self.draw_crossval_subplots()
        self.plot_label_and_set_xy_lim()
        plt.tight_layout()
        plt.pause(0.1)


class MetricsPlotterNode(Node):
    def __init__(self):
        super().__init__("plot_metric")
        self.metrics_plotter = MetricsPlotter()
        self.subscription = self.create_subscription(
            Float32MultiArray, "calibration_metrics", self.listener_callback, 10
        )

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
