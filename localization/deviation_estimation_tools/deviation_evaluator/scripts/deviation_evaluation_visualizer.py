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

import os
from pathlib import Path
from threading import Thread

from bag_load_utils import BagFileEvaluator
import matplotlib.pyplot as plt
import numpy as np
from plot_utils import plot_bag_compare
from plot_utils import plot_thresholds
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


class DeviationEvaluationVisualizer(Node):
    def __init__(self):
        super().__init__("deviation_evaluation_visualizer")
        self.declare_parameter("save_dir", "")

        save_dir = self.get_parameter("save_dir").get_parameter_value().string_value

        bagfile = Path(save_dir) / "ros2bag/ros2bag_0.db3"
        output_dir = Path(save_dir)

        bag_file_evaluator = BagFileEvaluator(str(bagfile), PARAMS)

        os.makedirs(output_dir / "body_frame", exist_ok=True)
        for thres in np.arange(0.1, 0.4, 0.05):
            plot_thresholds(
                bag_file_evaluator.calc_roc_curve_lateral(thres),
                bag_file_evaluator.results.lateral.lower_bound,
                thres,
                PARAMS["scale"],
                save_path=output_dir / "body_frame/thres2recall_{:.2f}.png".format(thres),
            )

        os.makedirs(output_dir / "long_radius", exist_ok=True)
        for thres in np.arange(0.1, 1.0, 0.05):
            plot_thresholds(
                bag_file_evaluator.calc_roc_curve_long_radius(thres),
                bag_file_evaluator.results.long_radius.lower_bound,
                thres,
                PARAMS["scale"],
                save_path=output_dir / "long_radius/thres2recall_{:.2f}.png".format(thres),
            )

        _ = plot_bag_compare(
            output_dir / "deviation_evaluator.png",
            bag_file_evaluator.results,
        )
        plt.show()
        print("Visualization completed! Press ctrl-C to exit.")


def main(args=None):
    rclpy.init(args=args)
    print("Loading rosbag. This may take a while...")
    node = DeviationEvaluationVisualizer()
    spin_thread = Thread(target=rclpy.spin, args=(node,))
    spin_thread.start()


if __name__ == "__main__":
    main()
