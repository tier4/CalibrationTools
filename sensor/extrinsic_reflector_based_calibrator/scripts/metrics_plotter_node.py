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


class Plotter:
    def __init__(self):
        self.fig, self.axes = plt.subplots(nrows=2, ncols=2, figsize=(15, 12))
        self.subplot0 = self.axes[0, 0]
        self.subplot1 = self.axes[0, 1]
        self.subplot2 = self.axes[1, 0]
        self.subplot3 = self.axes[1, 1]

        self.color_bo = "bo-"
        self.color_go = "go-"
        self.color_b = "b"
        self.color_g = "g"

        (
            self.num_of_reflectors,
            self.cv_distance_error,
            self.cv_yaw_error,
            self.calibration_distance_error,
            self.calibration_yaw_error,
        ) = (0, 0, 0, 0, 0)
        (
            self.prev_calibration_distance_error,
            self.prev_calibration_yaw_error,
            self.prev_cv_distance_error,
            self.prev_cv_yaw_error,
            self.prev_num_of_reflectors,
        ) = (0, 0, 0, 0, 0)
        (
            self.cv_distance_error_list,
            self.cv_yaw_error_list,
            self.calibration_distance_error_list,
            self.calibration_yaw_error_list,
            self.num_of_reflectors_list,
        ) = ([0], [0], [0], [0], [0])
        self.anno0, self.anno1, self.anno2, self.anno3 = None, None, None, None
        self.max_ylim0, self.max_ylim1, self.max_ylim2, self.max_ylim3 = 0, 0, 0, 0

        self.init_plot()
        plt.pause(0.1)

    def init_plot(self):
        self.subplot0.set_title("Cross val error: Distance")
        self.subplot0.set_xlabel("# reflector")
        self.subplot0.set_ylabel("Distance error (m)")

        self.subplot1.set_title("Cross val error: Yaw")
        self.subplot1.set_xlabel("# reflector")
        self.subplot1.set_ylabel("Yaw error (deg)")

        self.subplot2.set_title("Average error: Distance")
        self.subplot2.set_xlabel("# reflector")
        self.subplot2.set_ylabel("Distance error (m)")

        self.subplot3.set_title("Average error: Yaw")
        self.subplot3.set_xlabel("# reflector")
        self.subplot3.set_ylabel("Yaw error (deg)")

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
            self.prev_cv_distance_error, self.prev_num_of_reflectors, index = self.redraw_subplot(
                self.num_of_reflectors, self.cv_distance_error_list, self.subplot0, self.color_bo
            )
            self.prev_cv_yaw_error, _, _ = self.redraw_subplot(
                self.num_of_reflectors, self.cv_yaw_error_list, self.subplot1, self.color_go
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
            self.cv_distance_error_list.append(self.cv_distance_error)
            self.cv_yaw_error_list.append(self.cv_yaw_error)
            self.calibration_distance_error_list.append(self.calibration_distance_error)
            self.calibration_yaw_error_list.append(self.calibration_yaw_error)
            self.num_of_reflectors_list.append(self.num_of_reflectors)

    def update_xy_lim(self):
        self.max_ylim0 = (
            self.cv_distance_error if self.cv_distance_error > self.max_ylim0 else self.max_ylim0
        )
        self.max_ylim1 = self.cv_yaw_error if self.cv_yaw_error > self.max_ylim1 else self.max_ylim1
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

        self.subplot0.set_ylim(0, self.max_ylim0 + 0.1)
        self.subplot1.set_ylim(0, self.max_ylim1 + 0.1)
        self.subplot2.set_ylim(0, self.max_ylim2 + 0.1)
        self.subplot3.set_ylim(0, self.max_ylim3 + 0.1)

    def draw_lines(self):
        self.subplot0.plot(
            [self.prev_num_of_reflectors, self.num_of_reflectors],
            [self.prev_cv_distance_error, self.cv_distance_error],
            self.color_bo,
        )
        self.subplot1.plot(
            [self.prev_num_of_reflectors, self.num_of_reflectors],
            [self.prev_cv_yaw_error, self.cv_yaw_error],
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
            "%0.4f" % self.cv_distance_error,
            xy=(self.num_of_reflectors, self.cv_distance_error),
            color=self.color_b,
        )
        self.anno1 = self.subplot1.annotate(
            "%0.4f" % self.cv_yaw_error,
            xy=(self.num_of_reflectors, self.cv_yaw_error),
            color=self.color_g,
        )
        self.anno2 = self.subplot2.annotate(
            "%0.4f" % self.calibration_distance_error,
            xy=(self.num_of_reflectors, self.calibration_distance_error),
            color=self.color_b,
        )
        self.anno3 = self.subplot3.annotate(
            "%0.4f" % self.calibration_yaw_error,
            xy=(self.num_of_reflectors, self.calibration_yaw_error),
            color=self.color_g,
        )
        plt.pause(0.1)

    def draw_with_msg(self, msg):
        self.num_of_reflectors = msg.data[0]
        self.cv_distance_error = msg.data[1]
        self.cv_yaw_error = msg.data[2]
        self.calibration_distance_error = msg.data[3]
        self.calibration_yaw_error = 0 if math.isnan(msg.data[4]) else msg.data[4]

        self.check_if_deleted()
        self.update_xy_lim()
        self.remove_annotations()
        self.draw_lines()

        self.prev_cv_distance_error = self.cv_distance_error
        self.prev_cv_yaw_error = self.cv_yaw_error
        self.prev_calibration_distance_error = self.calibration_distance_error
        self.prev_calibration_yaw_error = self.calibration_yaw_error
        self.prev_num_of_reflectors = self.num_of_reflectors


class MetricsPlotter(Node):
    def __init__(self):
        super().__init__("plot_metric")
        self.plotter = Plotter()
        self.subscription = self.create_subscription(
            Float32MultiArray, "cross_validation_metrics", self.listener_callback, 10
        )

    def listener_callback(self, msg):
        self.plotter.draw_with_msg(msg)


def main(args=None):
    rclpy.init(args=args)
    metrics_plotter = MetricsPlotter()
    rclpy.spin(metrics_plotter)
    metrics_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
