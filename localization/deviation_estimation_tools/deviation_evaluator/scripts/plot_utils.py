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

import matplotlib.pyplot as plt
import numpy as np


def plot_thresholds(recall_list, lower_bound, threshold, scale, save_path=None):
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, xlabel="threshold [m]", ylabel="recall")
    ax.plot(recall_list[:, 0], recall_list[:, 1], label="Upper bound")
    ax.text(
        0.1,
        0.1,
        "Threshold lower bound = {:.3f} [m]".format(lower_bound),
        transform=ax.transAxes,
    )
    ax.grid()
    ax.legend()
    ax.set_title(
        "Recall for detecting localization anomalies (over {0:.3f} [m], {1}-sigma)".format(
            threshold, scale
        )
    )
    if save_path is not None:
        plt.savefig(save_path)
    plt.close()


def plot_duration_to_error(duration_to_error, save_path=None):
    if len(duration_to_error) > 0:
        fig = plt.figure(figsize=(8, 6))
        ax = fig.add_subplot(111, xlabel="duration [s]", ylabel="error [m]")
        ax.scatter(duration_to_error[:, 0], duration_to_error[:, 1], s=1)
        ax.grid()
        title = "Relationship between error (along body-y-axis) and duration of dead reckoning"
        ax.set_title(title)
        if save_path is not None:
            plt.savefig(save_path)
    else:
        print("NDT is not cut")


def plot_bag_compare(
    save_path,
    timestamp,
    error_long_radius,
    expected_error_long_radius,
    error_lateral,
    expected_error_lateral,
    error_frontal,
    expected_error_frontal,
    twist_list,
):

    lateral_warning_timestamps = timestamp[error_lateral > expected_error_lateral]
    frontal_warning_timestamps = timestamp[error_frontal > expected_error_frontal]

    timestamp_twist = twist_list[:, 0]
    velocity = twist_list[:, 1]
    yaw_rate = twist_list[:, 2]

    # Ignore the first 20 steps (=1 sec in 20 Hz) as this part may be noisy
    error_maximum = np.max(
        np.hstack(
            [
                expected_error_long_radius[20:],
                expected_error_lateral[20:],
                expected_error_frontal[20:],
            ]
        )
    )

    fig = plt.figure(figsize=(12, 12))

    ax1 = fig.add_subplot(411, xlabel="time [s]", ylabel="error [m]")
    ax1.plot(
        timestamp,
        error_long_radius,
        label="error",
    )
    ax1.plot(
        timestamp,
        expected_error_long_radius,
        label="expected error (long radius)",
        linestyle="--",
        color="gray",
    )
    ax1.legend(loc=[1.0, 0.7])
    ax1.set_ylim([0, error_maximum * 1.1])
    ax1.grid()

    # Plot error along body-frame-y-axis
    ax2 = fig.add_subplot(412, xlabel="time [s]", ylabel="error [m]")
    ax2.plot(
        timestamp,
        error_lateral,
        label="error along lateral direction",
    )

    ax2.vlines(
        lateral_warning_timestamps,
        0,
        max(expected_error_lateral),
        color=(1, 0.7, 0.7),
        label="Danger zone",
    )
    ax2.plot(
        timestamp,
        expected_error_lateral,
        label=r"expected error (3-sigma)",
        linestyle="--",
        color="gray",
    )
    ax2.legend(loc=[1.0, 0.7])
    ax2.set_ylim([0, error_maximum * 1.1])
    ax2.grid()

    # Plot error along body-frame-x-axis
    ax3 = fig.add_subplot(413, xlabel="time [s]", ylabel="error [m]")
    ax3.plot(
        timestamp,
        error_frontal,
        label="error along frontal direction",
    )
    ax3.vlines(
        frontal_warning_timestamps,
        0,
        max(expected_error_frontal),
        color=(1, 0.7, 0.7),
        label="Danger zone",
    )
    ax3.plot(
        timestamp,
        expected_error_frontal,
        label=r"expected error (3-sigma)",
        linestyle="--",
        color="gray",
    )
    ax3.legend(loc=[1.0, 0.7])
    ax3.set_ylim([0, error_maximum * 1.1])
    ax3.grid()

    # Plot velocity and yaw_rate
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
