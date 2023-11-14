#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2020 Tier IV, Inc.
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

from constants import THRESHOLD_FOR_INITIALIZED_ERROR
import matplotlib.pyplot as plt
import numpy as np


def plot_thresholds(recall_list, lower_bound, threshold, scale, save_path=None):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, xlabel="threshold [m]", ylabel="recall")
    ax.text(
        0.1, 0.1, "Threshold lower bound = {:.3f} [m]".format(lower_bound), transform=ax.transAxes
    )

    if recall_list is not None:
        ax.plot(recall_list[:, 0], recall_list[:, 1], label="Upper bound")
        ax.legend()
    else:
        ax.text(0.3, 0.5, "Error larger than {:.3f} [m] was not observed".format(threshold))
    ax.grid()
    ax.set_title(
        "Recall for detecting localization anomalies (over {0:.3f} [m], {1}-sigma)".format(
            threshold, scale
        )
    )
    if save_path is not None:
        plt.savefig(save_path)
    plt.close()


def plot_bag_compare(save_path, results):
    # Ignore the initial part larger than this value, since the values right after launch may diverge.
    is_smaller_than_thres = results.long_radius.expected_error < THRESHOLD_FOR_INITIALIZED_ERROR
    ignore_index = np.where(is_smaller_than_thres)[0][0]
    error_maximum = np.max(
        np.hstack(
            [
                results.long_radius.expected_error[ignore_index:],
                results.lateral.expected_error[ignore_index:],
                results.longitudinal.expected_error[ignore_index:],
            ]
        )
    )

    fig = plt.figure(figsize=(12, 12))
    ax1 = fig.add_subplot(411, xlabel="time [s]", ylabel="error [m]")
    plot_error_analysis(ax1, results.timestamps, results.long_radius, error_maximum)

    # Plot error along body-frame-y-axis
    ax2 = fig.add_subplot(412, xlabel="time [s]", ylabel="error [m]")
    plot_error_analysis(ax2, results.timestamps, results.lateral, error_maximum)

    # Plot error along body-frame-x-axis
    ax3 = fig.add_subplot(413, xlabel="time [s]", ylabel="error [m]")
    plot_error_analysis(ax3, results.timestamps, results.longitudinal, error_maximum)

    # Plot velocity and yaw_rate
    timestamp_twist = results.twist_list[:, 0]
    velocity = results.twist_list[:, 1]
    yaw_rate = results.twist_list[:, 2]

    ax4_vel = fig.add_subplot(414, xlabel="time [s]", ylabel="velocity [m/s]")
    ax4_vel.plot(timestamp_twist, velocity, label=r"velocity (left axis)", color="c")
    ax4_yaw_rate = ax4_vel.twinx()
    ax4_yaw_rate.plot(
        timestamp_twist,
        yaw_rate,
        label=r"yaw rate (right axis)",
        color="red",
    )
    handler1, label1 = ax4_vel.get_legend_handles_labels()
    handler2, label2 = ax4_yaw_rate.get_legend_handles_labels()
    ax4_vel.legend(handler1 + handler2, label1 + label2, loc=2, borderaxespad=0.0)

    plt.savefig(save_path, bbox_inches="tight")

    return fig


def plot_error_analysis(ax, timestamp, results, error_maximum):
    warning_timestamps = timestamp[results.error > results.expected_error]
    ax.vlines(
        warning_timestamps,
        0,
        max(results.expected_error),
        color=(1, 0.7, 0.7),
        label="Danger zone",
    )
    ax.plot(
        timestamp,
        results.error,
        label="error along {}".format(results.direction_name),
    )
    ax.plot(
        timestamp,
        results.expected_error,
        label=r"expected error (3-sigma)",
        linestyle="--",
        color="gray",
    )
    ax.legend(loc=[1.0, 0.7])
    ax.set_ylim([0, error_maximum * 1.1])
    ax.grid()
