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

# cspell:words lowerbound

from pathlib import Path
from threading import Thread

from bag_load_utils import BagFileEvaluator
import numpy as np
from plot_utils import plot_bag_compare
import rclpy
from rclpy.node import Node

PARAMS = {
    "twist_topic": "/deviation_evaluator/twist_estimator/twist_with_covariance",
    "pose_topic": "/deviation_evaluator/dead_reckoning/pose_estimator/pose_with_covariance",
    "ekf_gt_odom_topic": "/deviation_evaluator/ground_truth/ekf_localizer/kinematic_state",
    "ekf_dr_odom_topic": "/deviation_evaluator/dead_reckoning/ekf_localizer/kinematic_state",
    "scale": 3,
    "ndt_freq": 10,
}


def validate_threshold(
    recall: float, threshold: float, lowerbound: float, recall_threshold: float = 0.99
):
    if threshold < lowerbound:
        print("Threshold is too small for this vehicle.")
    elif recall == np.inf:
        print(
            f"No error larger than {threshold:.3f} [m] observed. Provide more data on deviation_evaluator."
        )
    elif recall > recall_threshold:
        print("Valid threshold!")
    else:
        print("Covariance seems to be too optimistic.")
        print("Consider increasing the covariances of the dead reckoning sensors.")

    if threshold > lowerbound:
        print(f"  [OK] lowerbound = {lowerbound:.3f} < {threshold:.3f} = threshold [m]")
    else:
        print(f"  [NG] lowerbound = {lowerbound:.3f} > {threshold:.3f} = threshold [m]")

    if recall == np.inf:
        print(f"  [NG] recall = {recall}")
    elif recall > recall_threshold:
        print(f"  [OK] recall = {recall:.3f} > {recall_threshold:.3f}")
    else:
        print(f"  [NG] recall = {recall:.3f} < {recall_threshold:.3f}")


class DeviationEvaluationVisualizer(Node):
    def __init__(self):
        super().__init__("deviation_evaluation_visualizer")
        self.declare_parameter("save_dir", "")
        self.declare_parameter("warn_ellipse_size", 0.0)
        self.declare_parameter("warn_ellipse_size_lateral_direction", 0.0)

        save_dir = self.get_parameter("save_dir").get_parameter_value().string_value
        threshold_long_radius = (
            self.get_parameter("warn_ellipse_size").get_parameter_value().double_value
        )
        threshold_lateral = (
            self.get_parameter("warn_ellipse_size_lateral_direction")
            .get_parameter_value()
            .double_value
        )

        bagfile = Path(save_dir) / "ros2bag/ros2bag_0.db3"
        output_dir = Path(save_dir)

        bag_file_evaluator = BagFileEvaluator(str(bagfile), PARAMS)

        recall_lateral = bag_file_evaluator.calc_recall_lateral(threshold_lateral)
        print("============================================================")
        print("Results for lateral error monitoring:")
        validate_threshold(
            recall_lateral, threshold_lateral, bag_file_evaluator.results.lateral.lower_bound
        )

        print("============================================================")
        print("Results for long radius error monitoring:")
        recall_long_radius = bag_file_evaluator.calc_recall_long_radius(threshold_long_radius)
        validate_threshold(
            recall_long_radius,
            threshold_long_radius,
            bag_file_evaluator.results.long_radius.lower_bound,
        )
        print("============================================================")

        _ = plot_bag_compare(
            output_dir / "deviation_evaluator.png",
            bag_file_evaluator.results,
        )
        # plt.show()
        print("Visualization completed! Press ctrl-C to exit.")


def main(args=None):
    rclpy.init(args=args)
    print("Loading rosbag. This may take a while...")
    node = DeviationEvaluationVisualizer()
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()


if __name__ == "__main__":
    main()
