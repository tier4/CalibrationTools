#!/usr/bin/env python3

# Copyright 2023 Tier IV, Inc.
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
        plt.gcf().canvas.set_window_title("Metrics plotter")

        self.color_bo = "bo-"
        self.color_go = "go-"
        self.color_b = "b"
        self.color_g = "g"

        (
            self.num_of_reflectors,
            self.crossval_distance_error,
            self.crossval_yaw_error,
            self.calibration_distance_error,
            self.calibration_yaw_error,
        ) = (0, 0, 0, 0, 0)
        (
            self.prev_calibration_distance_error,
            self.prev_calibration_yaw_error,
            self.prev_crossval_distance_error,
            self.prev_crossval_yaw_error,
            self.prev_num_of_reflectors,
        ) = (0, 0, 0, 0, 0)
        (
            self.crossval_distance_error_list,
            self.crossval_yaw_error_list,
            self.calibration_distance_error_list,
            self.calibration_yaw_error_list,
            self.num_of_reflectors_list,
        ) = ([0], [0], [0], [0], [0])
        self.anno0, self.anno1, self.anno2, self.anno3 = None, None, None, None
        self.max_ylim0, self.max_ylim1, self.max_ylim2, self.max_ylim3 = 0, 0, 0, 0
        self.m_to_cm = 0

        self.init_plot()
        plt.tight_layout()
        plt.pause(0.1)

    def init_plot(self):
        self.subplot0.set_title("cross-validation error: distance")
        self.subplot0.set_xlabel("number of tracks")
        self.subplot0.set_ylabel("distance error [cm]")

        self.subplot1.set_title("cross-validation error: yaw")
        self.subplot1.set_xlabel("number of tracks")
        self.subplot1.set_ylabel("yaw error (deg)")

        self.subplot2.set_title("average error: distance")
        self.subplot2.set_xlabel("number of tracks")
        self.subplot2.set_ylabel("distance error [cm]")

        self.subplot3.set_title("average error: yaw")
        self.subplot3.set_xlabel("number of tracks")
        self.subplot3.set_ylabel("yaw error (deg)")

        for ax in self.axes.flat:
            ax.xaxis.set_major_locator(plt.MaxNLocator(integer=True))

    def remove_annotation(self, annotation):
        if annotation is not None:
            annotation.remove()

    def remove_annotations(self):
        # remove the previous annotations
        self.remove_annotation(self.anno0)
        self.remove_annotation(self.anno1)
        self.remove_annotation(self.anno2)
        self.remove_annotation(self.anno3)

    def redraw_subplot(self, num_of_reflectors, error_list, subplot, color_o):
        subplot.clear()
        prev_error = 0
        prev_num = 0

        for index, num in enumerate(self.num_of_reflectors_list):
            if index == 0:
                continue
            if num < num_of_reflectors:
                # redraw the subplot
                subplot.plot(
                    [self.num_of_reflectors_list[index - 1], self.num_of_reflectors_list[index]],
                    [error_list[index - 1], error_list[index]],
                    color_o,
                )
                prev_error = error_list[index]
                prev_num = num
            else:
                break

        del error_list[index : len(error_list)]
        return prev_error, prev_num, index

    def check_if_deleted(self):
        if self.num_of_reflectors < self.num_of_reflectors_list[-1]:
            (
                self.prev_crossval_distance_error,
                self.prev_num_of_reflectors,
                index,
            ) = self.redraw_subplot(
                self.num_of_reflectors,
                self.crossval_distance_error_list,
                self.subplot0,
                self.color_bo,
            )
            self.prev_crossval_yaw_error, _, _ = self.redraw_subplot(
                self.num_of_reflectors, self.crossval_yaw_error_list, self.subplot1, self.color_go
            )
            self.prev_calibration_distance_error, _, _ = self.redraw_subplot(
                self.num_of_reflectors,
                self.calibration_distance_error_list,
                self.subplot2,
                self.color_bo,
            )
            self.prev_calibration_yaw_error, _, _ = self.redraw_subplot(
                self.num_of_reflectors,
                self.calibration_yaw_error_list,
                self.subplot3,
                self.color_go,
            )
            del self.num_of_reflectors_list[index : len(self.num_of_reflectors_list)]
            self.init_plot()

        if self.num_of_reflectors != 0:
            self.crossval_distance_error_list.append(self.crossval_distance_error)
            self.crossval_yaw_error_list.append(self.crossval_yaw_error)
            self.calibration_distance_error_list.append(self.calibration_distance_error)
            self.calibration_yaw_error_list.append(self.calibration_yaw_error)
            self.num_of_reflectors_list.append(self.num_of_reflectors)

    def update_xy_lim(self):
        self.max_ylim0 = (
            self.crossval_distance_error
            if self.crossval_distance_error > self.max_ylim0
            else self.max_ylim0
        )
        self.max_ylim1 = (
            self.crossval_yaw_error if self.crossval_yaw_error > self.max_ylim1 else self.max_ylim1
        )
        self.max_ylim2 = (
            self.calibration_distance_error
            if self.calibration_distance_error > self.max_ylim2
            else self.max_ylim2
        )
        self.max_ylim3 = (
            self.calibration_yaw_error
            if self.calibration_yaw_error > self.max_ylim3
            else self.max_ylim3
        )

        # make the plot dynamic
        xlim = 1 if self.num_of_reflectors < 1 else self.num_of_reflectors
        self.subplot0.set_xlim(0, xlim)
        self.subplot1.set_xlim(0, xlim)
        self.subplot2.set_xlim(0, xlim)
        self.subplot3.set_xlim(0, xlim)

        self.subplot0.set_ylim(0, self.max_ylim0 + 5)
        self.subplot1.set_ylim(0, self.max_ylim1 + 0.1)
        self.subplot2.set_ylim(0, self.max_ylim2 + 5)
        self.subplot3.set_ylim(0, self.max_ylim3 + 0.1)

    def draw_lines(self):
        self.subplot0.plot(
            [self.prev_num_of_reflectors, self.num_of_reflectors],
            [self.prev_crossval_distance_error, self.crossval_distance_error],
            self.color_bo,
        )
        self.subplot1.plot(
            [self.prev_num_of_reflectors, self.num_of_reflectors],
            [self.prev_crossval_yaw_error, self.crossval_yaw_error],
            self.color_go,
        )
        self.subplot2.plot(
            [self.prev_num_of_reflectors, self.num_of_reflectors],
            [self.prev_calibration_distance_error, self.calibration_distance_error],
            self.color_bo,
        )
        self.subplot3.plot(
            [self.prev_num_of_reflectors, self.num_of_reflectors],
            [self.prev_calibration_yaw_error, self.calibration_yaw_error],
            self.color_go,
        )

        self.anno0 = self.subplot0.annotate(
            f"{self.crossval_distance_error:.2f}",
            xy=(self.num_of_reflectors, self.crossval_distance_error),
            color=self.color_b,
        )
        self.anno1 = self.subplot1.annotate(
            f"{self.crossval_yaw_error:.2f}",
            xy=(self.num_of_reflectors, self.crossval_yaw_error),
            color=self.color_g,
        )
        self.anno2 = self.subplot2.annotate(
            f"{self.calibration_distance_error:.2f}",
            xy=(self.num_of_reflectors, self.calibration_distance_error),
            color=self.color_b,
        )
        self.anno3 = self.subplot3.annotate(
            f"{self.calibration_yaw_error:.2f}",
            xy=(self.num_of_reflectors, self.calibration_yaw_error),
            color=self.color_g,
        )
        plt.tight_layout()
        plt.pause(0.1)

    def draw_with_msg(self, msg):
        self.num_of_reflectors = msg.data[0]
        # changing from meters to centimeters
        self.crossval_distance_error = msg.data[1] * self.m_to_cm
        self.crossval_yaw_error = msg.data[2]
        self.calibration_distance_error = msg.data[3] * self.m_to_cm
        self.calibration_yaw_error = 0 if math.isnan(msg.data[4]) else msg.data[4]

        self.check_if_deleted()
        self.update_xy_lim()
        self.remove_annotations()
        self.draw_lines()

        self.prev_crossval_distance_error = self.crossval_distance_error
        self.prev_crossval_yaw_error = self.crossval_yaw_error
        self.prev_calibration_distance_error = self.calibration_distance_error
        self.prev_calibration_yaw_error = self.calibration_yaw_error
        self.prev_num_of_reflectors = self.num_of_reflectors


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
