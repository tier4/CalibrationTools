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


import argparse
import bisect
import os
from pathlib import Path
import sqlite3
from threading import Thread

from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from tqdm import tqdm

TWIST_TOPIC = "/deviation_evaluator/twist_estimator/twist_with_covariance"
POSE_TOPIC = "/deviation_evaluator/dead_reckoning/pose_estimator/pose_with_covariance"
NDT_POSE_TOPIC = "/localization/pose_estimator/pose_with_covariance"
EKF_GT_ODOM_TOPIC = "/deviation_evaluator/ground_truth/ekf_localizer/kinematic_state"
EKF_DR_ODOM_TOPIC = "/deviation_evaluator/dead_reckoning/ekf_localizer/kinematic_state"
SCALE = 3
NDT_FREQ = 10


def calc_stddev_rotated(P, theta):
    e_vec = np.array([[np.cos(theta)], [np.sin(theta)]])
    d = np.sqrt(np.dot(np.dot(e_vec.T, P), e_vec) / np.linalg.det(P))
    return d[0][0]


class BagFileParser:
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of, type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of, name_of, type_of in topics_data}
        self.topic_msg_message = {
            name_of: get_message(type_of) for id_of, name_of, type_of in topics_data
        }

    def __del__(self):
        self.conn.close()

    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        return [
            (timestamp, deserialize_message(data, self.topic_msg_message[topic_name]))
            for timestamp, data in rows
        ]


class EKFBagFileParser(BagFileParser):
    def __init__(self, bag_file):
        super().__init__(bag_file)

    def get_messages(self, topic_name):
        topic_id = self.topic_id[topic_name]
        rows = self.cursor.execute(
            "SELECT timestamp, data FROM messages WHERE topic_id = {}".format(topic_id)
        ).fetchall()
        sample = deserialize_message(rows[0][1], self.topic_msg_message[topic_name])
        if isinstance(sample, PoseWithCovarianceStamped):
            return self.parse_pose_with_covariance_stamped(rows, topic_name)
        elif isinstance(sample, TwistWithCovarianceStamped):
            return self.parse_twist_with_covariance_stamped(rows, topic_name)
        elif isinstance(sample, Odometry):
            return self.parse_odometry(rows, topic_name)
        else:
            print(topic_name + " not found.")
            return []

    def parse_pose_with_covariance_stamped(self, rows, topic_name):
        pose_list, pose_cov_list = [], []
        for timestamp, data in rows:
            msg = deserialize_message(data, self.topic_msg_message[topic_name])
            R = Rotation.from_quat(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ]
            )
            _, _, yaw = R.as_euler("xyz", degrees=False)
            pose_list.append(
                [
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    yaw,
                ]
            )
            pose_cov_list.append(msg.pose.covariance.reshape(6, 6)[(0, 1, 5), :][:, (0, 1, 5)])
        sorted_idxs = np.argsort(np.array(pose_list)[:, 0])
        pose_list = np.array(pose_list)[sorted_idxs]
        pose_cov_list = np.array(pose_cov_list)[sorted_idxs]
        pose_list = np.array(pose_list)
        pose_cov_list = np.array(pose_cov_list)
        return pose_list, pose_cov_list

    def parse_twist_with_covariance_stamped(self, rows, topic_name):
        twist_list = []
        for timestamp, data in tqdm(rows, desc="Loading rosbag"):
            msg = deserialize_message(data, self.topic_msg_message[topic_name])
            twist_list.append(
                [
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    msg.twist.twist.linear.x,
                    msg.twist.twist.angular.z,
                ]
            )
        twist_list = np.array(twist_list)
        return np.array(sorted(twist_list, key=lambda x: x[0]))

    def parse_odometry(self, rows, topic_name):
        pose_list, pose_cov_list = [], []
        for timestamp, data in rows:
            msg = deserialize_message(data, self.topic_msg_message[topic_name])
            R = Rotation.from_quat(
                [
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                ]
            )
            _, _, yaw = R.as_euler("xyz", degrees=False)
            pose_list.append(
                [
                    msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                    yaw,
                ]
            )
            pose_cov_list.append(msg.pose.covariance.reshape(6, 6)[(0, 1, 5), :][:, (0, 1, 5)])
        sorted_idxs = np.argsort(np.array(pose_list)[:, 0])
        pose_list = np.array(pose_list)[sorted_idxs]
        pose_cov_list = np.array(pose_cov_list)[sorted_idxs]
        pose_list = np.array(pose_list)
        pose_cov_list = np.array(pose_cov_list)
        return pose_list, pose_cov_list


class BagFileEvaluator:
    def __init__(self, bagfile, use_normal_ekf=False, bagfile_base=None):
        bag_parser = EKFBagFileParser(bagfile)

        self.pose_list, _ = bag_parser.get_messages(POSE_TOPIC)
        self.ndt_pose_list, _ = bag_parser.get_messages(NDT_POSE_TOPIC)
        self.ekf_gt_pose_list, self.ekf_gt_pose_cov_list = bag_parser.get_messages(
            EKF_GT_ODOM_TOPIC
        )
        self.ekf_pose_list, self.ekf_pose_cov_list = bag_parser.get_messages(EKF_DR_ODOM_TOPIC)
        self.twist_list = bag_parser.get_messages(TWIST_TOPIC)

        self.ekf_gt_pose_list_interpolated, self.allowed_idxs = self.calc_interpolate()
        self.error_vec_xy, self.error_vec, self.error_vec_body_frame = self.calc_errors()

        (
            self.stddev_frontal_2d,
            self.stddev_lateral_2d,
            self.mahalanobis_2d,
        ) = self.calc_body_frame_length(self.ekf_pose_cov_list, self.ekf_pose_list, only_lim=True)
        self.stddev_frontal_2d_gt, self.stddev_lateral_2d_gt, _ = self.calc_body_frame_length(
            self.ekf_gt_pose_cov_list, self.ekf_gt_pose_list, only_lim=False
        )

        self.stddev_long_2d, self.stddev_short_2d = self.calc_long_short_radius(
            self.ekf_pose_cov_list
        )
        self.stddev_long_2d_gt, self.stddev_short_2d_gt = self.calc_long_short_radius(
            self.ekf_gt_pose_cov_list
        )

    def calc_interpolate(self):
        gt_timestamps = self.ekf_gt_pose_list[:, 0].tolist()
        timestamps = self.ekf_pose_list[:, 0].tolist()
        ekf_gt_pose_list_interpolated = []

        for i in range(len(timestamps)):
            t = self.ekf_pose_list[i, 0]
            idx = bisect.bisect_left(gt_timestamps, t, 0, -1)
            if idx < 0:
                interpolated = self.ekf_gt_pose_list[0, 1:3]
            elif idx >= len(gt_timestamps):
                interpolated = self.ekf_gt_pose_list[-1, 1:3]
            else:
                t0 = gt_timestamps[idx - 1]
                t1 = gt_timestamps[idx]
                x0 = self.ekf_gt_pose_list[idx - 1, 1:3]
                x1 = self.ekf_gt_pose_list[idx, 1:3]

                interpolated = ((t1 - t) * x0 + (t - t0) * x1) / (t1 - t0)
            ekf_gt_pose_list_interpolated.append([t, interpolated[0], interpolated[1]])
        ekf_gt_pose_list_interpolated = np.array(ekf_gt_pose_list_interpolated)

        allowed_idxs = np.array(timestamps) < min(timestamps[-1], gt_timestamps[-1]) * (
            np.array(timestamps) > max(timestamps[0], gt_timestamps[0])
        )
        return ekf_gt_pose_list_interpolated, allowed_idxs

    def calc_errors(self):
        error_vec_xy = self.ekf_gt_pose_list_interpolated[:, 1:3] - self.ekf_pose_list[:, 1:3]
        error = np.linalg.norm(error_vec_xy, axis=1)

        # calculate error along long & short axis of confidence ellipse
        long_axis_direction = np.linalg.eig(self.ekf_pose_cov_list[:, :2, :2])[1][:, :, 1]
        error_vec = np.abs((error * long_axis_direction.T).T)

        # calculate body_frame error
        cos_val = (
            error_vec_xy[:, 0] * np.cos(self.ekf_pose_list[:, 4])
            + error_vec_xy[:, 1] * np.sin(self.ekf_pose_list[:, 4])
        ) / error
        sin_val = np.sqrt(1 - cos_val * cos_val)
        error_vec_body_frame = np.abs((error * np.array([cos_val, sin_val])).T)
        return error_vec_xy, error_vec, error_vec_body_frame

    def calc_body_frame_length(self, cov_list, pose_list, only_lim=False):
        inv_2d = np.linalg.inv(cov_list[:, :2, :2])
        mahalanobis_2d = []
        stddev_frontal_2d = []
        stddev_lateral_2d = []

        for i in range(len(cov_list)):
            cov_matrix = inv_2d[i]
            yaw = pose_list[i, 4]
            stddev_frontal_2d.append(calc_stddev_rotated(cov_matrix, yaw - np.pi / 2))
            stddev_lateral_2d.append(calc_stddev_rotated(cov_matrix, yaw))
            if only_lim:
                error = self.error_vec[i]
                val = np.dot(np.dot(error, cov_matrix), error)
                mahalanobis_2d.append(val)

        mahalanobis_2d = np.array(mahalanobis_2d)
        stddev_frontal_2d = np.array(stddev_frontal_2d)
        stddev_lateral_2d = np.array(stddev_lateral_2d)
        return stddev_frontal_2d, stddev_lateral_2d, mahalanobis_2d

    def calc_long_short_radius(self, cov_list):
        cov_eig_val_list = np.linalg.eig(cov_list[:, :2, :2])[0]
        cov_eig_val_list[cov_eig_val_list < 0] = 1e-5
        stddev_long_2d = np.sqrt(np.max(cov_eig_val_list, axis=1))
        stddev_short_2d = np.sqrt(np.min(cov_eig_val_list, axis=1))
        return stddev_long_2d, stddev_short_2d

    def calc_roc_curve_body_frame(self, a_th):
        recall_list = self.calc_roc_curve(
            a_th, self.error_vec_body_frame[:, 1], self.stddev_lateral_2d
        )
        return recall_list

    def calc_roc_curve_long_radius(self, a_th):
        recall_list = self.calc_roc_curve(
            a_th, np.linalg.norm(self.error_vec, axis=1), self.stddev_long_2d
        )
        return recall_list

    def calc_roc_curve(self, a_th, error, stddev):
        a = error
        b = stddev * SCALE

        Aeq1 = a > a_th

        recall_list = []
        b_th_list = np.arange(0, 0.8, 0.02)
        for b_th in b_th_list:
            Beq1 = b > b_th
            recall = np.sum(Aeq1 & Beq1) * 1.0 / np.sum(Aeq1)
            recall_list.append([b_th, recall])
        return np.array(recall_list)

    def get_duration_to_error(
        self,
    ):
        duration_to_error = []
        for timestamp, error in zip(self.ekf_pose_list[:, 0], self.error_vec_body_frame[:, 1]):
            idx = bisect.bisect_left(self.pose_list[:, 0], timestamp)
            if idx > 0 and error < 2:
                duration = timestamp - self.pose_list[idx - 1, 0]
                if duration > 5.0 / NDT_FREQ:  # Only count if NDT hasn't come for 5 steps.
                    duration_to_error.append([duration, error])
        return np.array(duration_to_error)

    # ToDo: Calculation algorithm for threshold lower bound should be updated. Currently the
    # threshold lower bound is determined using EKF output based on default parameter in
    # Autoware.
    def calc_thres_lower_bound(self, how):
        if how == "body_frame":
            b = self.stddev_lateral_2d_gt * SCALE
        elif how == "long_radius":
            b = self.stddev_long_2d_gt * SCALE

        thres_lower_bound = 0
        for idx in range(len(b)):
            if idx < 100:  # ignore first 10[s]
                continue
            timestamp = self.ekf_gt_pose_list[idx, 0]
            idx_ndt = bisect.bisect_left(self.ndt_pose_list[:, 0], timestamp)
            if timestamp - self.ndt_pose_list[idx_ndt - 3, 0] < 4.0 / NDT_FREQ:
                thres_lower_bound = max(thres_lower_bound, b[idx])
        return thres_lower_bound

    def plot_thresholds(self, a_th, save_path=None, how="body_frame"):
        if how == "body_frame":
            recall_list = self.calc_roc_curve_body_frame(a_th)
            lower_bound = self.calc_thres_lower_bound(how=how)
        elif how == "long_radius":
            recall_list = self.calc_roc_curve_long_radius(a_th)
            lower_bound = self.calc_thres_lower_bound(how=how)

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
                a_th, SCALE
            )
        )
        if save_path is not None:
            plt.savefig(save_path)
        plt.close()

    def plot_duration_to_error(self, save_path=None):
        duration_to_error = self.get_duration_to_error()
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

    def plot_bag_compare(self, save_path):
        # Ignore the first 20 steps (=1 sec in 20 Hz) as this part may be noisy
        error_maximum = np.max(
            np.hstack(
                [
                    self.stddev_long_2d[self.allowed_idxs][20:] * 3,
                    self.stddev_lateral_2d[self.allowed_idxs][20:] * 3,
                    self.stddev_frontal_2d[self.allowed_idxs][20:] * 3,
                ]
            )
        )

        fig = plt.figure(figsize=(12, 12))

        ax1 = fig.add_subplot(411, xlabel="time [s]", ylabel="error [m]")
        ax1.plot(
            self.ekf_pose_list[self.allowed_idxs, 0],
            np.linalg.norm(self.error_vec, axis=1)[self.allowed_idxs],
            label="error",
        )
        ax1.plot(
            self.ekf_pose_list[self.allowed_idxs, 0],
            self.stddev_long_2d[self.allowed_idxs] * 3,
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
            self.ekf_pose_list[self.allowed_idxs, 0],
            self.error_vec_body_frame[self.allowed_idxs, 1],
            label="error along body-frame-y-axis",
        )

        limit = self.stddev_lateral_2d[self.allowed_idxs] * 3
        idxs = self.error_vec_body_frame[self.allowed_idxs, 1] > limit
        ax2.vlines(
            self.ekf_pose_list[self.allowed_idxs, :][idxs, 0],
            0,
            max(limit),
            color=(1, 0.7, 0.7),
            label="Danger zone",
        )
        ax2.plot(
            self.ekf_pose_list[self.allowed_idxs, 0],
            limit,
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
            self.ekf_pose_list[self.allowed_idxs, 0],
            self.error_vec_body_frame[self.allowed_idxs, 0],
            label="error along body-frame-x-axis",
        )
        limit = self.stddev_frontal_2d[self.allowed_idxs] * 3
        idxs = self.error_vec_body_frame[self.allowed_idxs, 0] > limit
        ax3.vlines(
            self.ekf_pose_list[self.allowed_idxs, :][idxs, 0],
            0,
            max(limit),
            color=(1, 0.7, 0.7),
            label="Danger zone",
        )
        ax3.plot(
            self.ekf_pose_list[self.allowed_idxs, 0],
            limit,
            label=r"expected error (3-sigma)",
            linestyle="--",
            color="gray",
        )
        ax3.legend(loc=[1.0, 0.7])
        ax3.set_ylim([0, error_maximum * 1.1])
        ax3.grid()

        # Plot velocity and yaw_rate
        ax4_vel = fig.add_subplot(414, xlabel="time [s]", ylabel="velocity [m/s]")
        ax4_vel.plot(
            self.twist_list[:, 0], self.twist_list[:, 1], label=r"velocity (left axis)", color="c"
        )
        ax4_yaw_rate = ax4_vel.twinx()
        ax4_yaw_rate.plot(
            self.twist_list[:, 0],
            self.twist_list[:, 2],
            label=r"yaw rate (right axis)",
            color="red",
        )
        handler1, label1 = ax4_vel.get_legend_handles_labels()
        handler2, label2 = ax4_yaw_rate.get_legend_handles_labels()
        ax4_vel.legend(handler1 + handler2, label1 + label2, loc=2, borderaxespad=0.0)

        plt.savefig(save_path, bbox_inches="tight")

        return fig


class DeviationEvaluationVisualizer(Node):
    def __init__(self):
        super().__init__("deviation_evaluation_visualizer")
        self.declare_parameter("save_dir", "")

        save_dir = self.get_parameter("save_dir").get_parameter_value().string_value

        bagfile = Path(save_dir) / "ros2bag/ros2bag_0.db3"
        output_dir = Path(save_dir)

        bag_analyzer = BagFileEvaluator(str(bagfile))

        os.makedirs(output_dir / "body_frame", exist_ok=True)
        for thres in np.arange(0.1, 0.4, 0.05):
            bag_analyzer.plot_thresholds(
                thres,
                save_path=output_dir / "body_frame/thres2recall_{:.2f}.png".format(thres),
                how="body_frame",
            )

        # ToDo: This causes error when cut=0
        bag_analyzer.plot_duration_to_error(save_path=output_dir / "body_frame/duration2error.png")

        os.makedirs(output_dir / "long_radius", exist_ok=True)
        for thres in np.arange(0.1, 1.0, 0.05):
            bag_analyzer.plot_thresholds(
                thres,
                save_path=output_dir / "long_radius/thres2recall_{:.2f}.png".format(thres),
                how="long_radius",
            )

        fig = bag_analyzer.plot_bag_compare(output_dir / "deviation_evaluator.png")
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
