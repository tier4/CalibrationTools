#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc. All rights reserved.
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
from threading import Thread

from matplotlib import animation
import matplotlib.pyplot as plt
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

plt.style.use("dark_background")


class CorrData:
    def __init__(self, name, node):
        self.print_debug = False
        self.input_received = False
        self.response_received = False
        self.correlation_received = False
        self.log_corr = np.arange(1, 0.0)
        self.cross_correlation = np.arange(1, 0.0)
        self.input = np.arange(1, 0.0)
        self.response = np.arange(1, 0.0)
        self.estimated = np.arange(1, 0.0)
        self._node = node

        self.sub_input = node.create_subscription(
            Float32MultiArray, "/debug_values/" + name + "_input", self.CallBackInput, 10
        )
        self.sub_response = node.create_subscription(
            Float32MultiArray, "/debug_values/" + name + "_response", self.CallBackResponse, 10
        )
        self.sub_estimated = node.create_subscription(
            Float32MultiArray, "/debug_values/" + name + "_estimated", self.CallBackEstimated, 10
        )
        self.sub_correlation = node.create_subscription(
            Float64MultiArray, "/debug_values/" + name + "_correlation", self.CallBackCorr, 10
        )

    def CallBackCorr(self, msg):
        self.cross_correlation = msg.data
        self.correlation_received = True
        # self.log_corr = np.log10(self.cross_correlation)
        if self.print_debug:
            print(self.cross_correlation[::1])

    def CallBackInput(self, msg):
        self.input = msg.data
        self.input_received = True
        if self.print_debug:
            self._node.get_logger().info("in")
            print(self.input[::1])

    def CallBackResponse(self, msg):
        self.response = msg.data
        self.response_received = True
        if self.print_debug:
            self._node.get_logger().info("resp")
            print(self.response[::1])

    def CallBackEstimated(self, msg):
        self.estimated = msg.data
        self.estimated_received = True
        if self.print_debug:
            self._node.get_logger().info("est")


class PlotData:
    def __init__(self):
        self.ax_corr = None
        self.ax_val = None
        self.im_corr = None
        self.im_input = None
        self.im_response = None
        self.im_estimated = None


class TimeDelayAnimator(Node):
    def __init__(self):
        super().__init__("correlation_visualizer")
        self.debugMode()

    def debugMode(self):
        self.accel_plt = PlotData()  # accel
        self.brake_plt = PlotData()  # brake
        self.steer_plt = PlotData()  # steer
        self.fig, (
            self.accel_plt.ax_corr,
            self.brake_plt.ax_corr,
            self.steer_plt.ax_corr,
        ) = plt.subplots(3, 1, figsize=(6, 12))

        self.accel_data = CorrData("accel", self)
        self.brake_data = CorrData("brake", self)
        self.steer_data = CorrData("steer", self)
        self.ani = animation.FuncAnimation(
            self.fig, self.animationCallback, interval=200, blit=True
        )
        min_max = [0, 1000]
        self.setPlotGraph(self.accel_plt, "accel", min_max, [0, 1])
        self.setPlotGraph(self.brake_plt, "brake", min_max, [0, 1])
        self.setPlotGraph(self.steer_plt, "steer", min_max, [-1, 1])
        return

    def setPlotGraph(self, plotter, name, min_max, range_list):
        plotter.ax_val = plotter.ax_corr.twinx()
        plotter.ax_corr.set_xlim(min_max)
        plotter.ax_corr.set_ylim(range_list)
        plotter.ax_val.set_xlim(min_max)
        plotter.ax_val.set_ylim(range_list)
        (plotter.im_corr,) = plotter.ax_corr.plot(
            [], [], label="0: cross correlation", color="red", marker=""
        )
        (plotter.im_input,) = plotter.ax_val.plot(
            [], [], label="1: input (right axis)", color="green", marker=""
        )
        (plotter.im_response,) = plotter.ax_val.plot(
            [], [], label="2: response (right axis)", color="purple", marker=""
        )
        (plotter.im_estimated,) = plotter.ax_val.plot(
            [], [], label="3: estimated (right axis)", color="orange", marker=""
        )
        handler1, label1 = plotter.ax_corr.get_legend_handles_labels()
        handler2, label2 = plotter.ax_val.get_legend_handles_labels()
        plotter.ax_corr.legend(handler1 + handler2, label1 + label2, loc=2, borderaxespad=0.0)
        plotter.ax_corr.set_xlabel(name + " correlation index / time")
        plotter.ax_corr.set_ylabel(name + " correlation")
        plotter.ax_val.set_ylabel(name + "  value")
        return

    def animationCallback(self, data):
        try:
            self.timerCallback()
        except KeyboardInterrupt:
            exit(0)
        return (
            self.accel_plt.im_corr,
            self.accel_plt.im_input,
            self.accel_plt.im_response,
            self.accel_plt.im_estimated,
            self.brake_plt.im_corr,
            self.brake_plt.im_input,
            self.brake_plt.im_response,
            self.brake_plt.im_estimated,
            self.steer_plt.im_corr,
            self.steer_plt.im_input,
            self.steer_plt.im_response,
            self.steer_plt.im_estimated,
        )

    def setData(self, data, plotter, resample_rate=1):
        corr_data_y = copy.deepcopy(np.array(data.cross_correlation[::resample_rate]))
        corr_data_x = np.arange(len(corr_data_y))
        input_data_y = copy.deepcopy(np.array(data.input[::-resample_rate]))
        input_data_x = np.arange(len(input_data_y))
        response_data_y = copy.deepcopy(np.array(data.response[::-resample_rate]))
        response_data_x = np.arange(len(response_data_y))
        estimated_data_y = copy.deepcopy(np.array(data.estimated[::-resample_rate]))
        estimated_data_x = np.arange(len(estimated_data_y))
        if len(corr_data_x) != len(corr_data_y):
            print("size mismatch for I/O")
            return
        plotter.im_corr.set_data(corr_data_x, corr_data_y)
        plotter.im_input.set_data(input_data_x, input_data_y)
        plotter.im_response.set_data(response_data_x, response_data_y)
        plotter.im_estimated.set_data(estimated_data_x, estimated_data_y)
        return

    def timerCallback(self):
        # print(len(input_x), len(response_x), len(corr_data))
        self.setData(self.accel_data, self.accel_plt)
        self.setData(self.brake_data, self.brake_plt)
        self.setData(self.steer_data, self.steer_plt)
        return

    def closeFigure(self):
        plt.close(self.fig)


def main(args=None):
    rclpy.init(args=args)
    node = TimeDelayAnimator()
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()
    plt.show()


if __name__ == "__main__":
    main()
