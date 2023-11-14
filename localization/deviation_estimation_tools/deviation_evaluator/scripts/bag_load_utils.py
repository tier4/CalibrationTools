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


import bisect
import dataclasses
import sqlite3

from constants import THRESHOLD_FOR_INITIALIZED_ERROR
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation
from tqdm import tqdm


@dataclasses.dataclass
class ErrorResults:
    direction_name: str
    error: np.ndarray
    expected_error: np.ndarray
    lower_bound: float = None


@dataclasses.dataclass
class DeviationEvaluatorResults:
    long_radius: ErrorResults
    lateral: ErrorResults
    longitudinal: ErrorResults
    timestamps: np.ndarray
    ndt_timestamps: np.ndarray
    twist_list: np.ndarray


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
        return pose_list, pose_cov_list


class BagFileEvaluator:
    def __init__(self, bagfile, params, use_normal_ekf=False, bagfile_base=None):
        bag_parser = EKFBagFileParser(bagfile)

        pose_list, _ = bag_parser.get_messages(params["pose_topic"])
        ekf_gt_pose_list, ekf_gt_pose_cov_list = bag_parser.get_messages(
            params["ekf_gt_odom_topic"]
        )
        ekf_dr_pose_list, ekf_dr_pose_cov_list = bag_parser.get_messages(
            params["ekf_dr_odom_topic"]
        )

        ekf_gt_pose_list_interpolated, valid_idxs = calc_interpolate(
            ekf_gt_pose_list[:, 1:3],
            ekf_gt_pose_list[:, 0].tolist(),
            ekf_dr_pose_list[:, 0].tolist(),
        )

        long_axis_angles = get_long_axis_angles_from_covs(ekf_dr_pose_cov_list[:, :2, :2])
        errors = ekf_gt_pose_list_interpolated[:, 1:3] - ekf_dr_pose_list[:, 1:3]
        errors_along_elliptical_axis = transform_errors(errors, long_axis_angles)
        errors_along_body_frame = transform_errors(errors, ekf_dr_pose_list[:, 4])

        stddev_longitudinal_2d, stddev_lateral_2d = calc_body_frame_length(
            ekf_dr_pose_cov_list,
            ekf_dr_pose_list,
        )
        stddev_longitudinal_2d_gt, stddev_lateral_2d_gt = calc_body_frame_length(
            ekf_gt_pose_cov_list,
            ekf_gt_pose_list,
        )

        stddev_long_2d, stddev_short_2d = calc_long_short_radius(ekf_dr_pose_cov_list)
        stddev_long_2d_gt, stddev_short_2d_gt = calc_long_short_radius(ekf_gt_pose_cov_list)

        ignore_index = np.where(stddev_long_2d_gt < THRESHOLD_FOR_INITIALIZED_ERROR)[0][0]

        long_radius_results = ErrorResults(
            "long_radius",
            np.linalg.norm(errors_along_elliptical_axis, axis=1)[valid_idxs],
            stddev_long_2d[valid_idxs] * params["scale"],
            np.max(stddev_long_2d_gt[ignore_index:]) * params["scale"],
        )
        lateral_results = ErrorResults(
            "lateral",
            np.abs(errors_along_body_frame[valid_idxs, 1]),
            stddev_lateral_2d[valid_idxs] * params["scale"],
            np.max(stddev_lateral_2d_gt[ignore_index:]) * params["scale"],
        )
        longitudinal_results = ErrorResults(
            "longitudinal",
            np.abs(errors_along_body_frame[valid_idxs, 0]),
            stddev_longitudinal_2d[valid_idxs] * params["scale"],
        )

        self.results = DeviationEvaluatorResults(
            long_radius_results,
            lateral_results,
            longitudinal_results,
            ekf_dr_pose_list[valid_idxs, 0],
            pose_list[:, 0],
            bag_parser.get_messages(params["twist_topic"]),
        )

    # def calc_roc_curve_lateral(self, threshold):
    #     recall_list = calc_roc_curve(threshold, self.results.lateral)
    #     return recall_list

    # def calc_roc_curve_long_radius(self, threshold):
    #     recall_list = calc_roc_curve(threshold, self.results.long_radius)
    #     return recall_list

    def calc_recall_lateral(self, threshold):
        positives = self.results.lateral.error > threshold
        if np.sum(positives) == 0:
            return np.inf
        true_positives = positives & (self.results.lateral.expected_error > threshold)
        return np.sum(true_positives) / np.sum(positives)

    def calc_recall_long_radius(self, threshold):
        positives = self.results.long_radius.error > threshold
        if np.sum(positives) == 0:
            return np.inf
        true_positives = positives & (self.results.long_radius.expected_error > threshold)
        return np.sum(true_positives) / np.sum(positives)


def calc_interpolate(poses, timestamps, timestamps_target):
    poses_interpolated = []

    for i in range(len(timestamps_target)):
        t = timestamps_target[i]
        idx = bisect.bisect_left(timestamps, t, 0, -1)
        if idx < 0:
            interpolated = poses[0]
        elif idx >= len(timestamps):
            interpolated = poses[-1]
        else:
            t0 = timestamps[idx - 1]
            t1 = timestamps[idx]
            x0 = poses[idx - 1]
            x1 = poses[idx]

            interpolated = ((t1 - t) * x0 + (t - t0) * x1) / (t1 - t0)
        poses_interpolated.append([t, interpolated[0], interpolated[1]])
    poses_interpolated = np.array(poses_interpolated)

    valid_idxs = np.array(timestamps_target) < min(timestamps[-1], timestamps_target[-1]) * (
        np.array(timestamps_target) > max(timestamps[0], timestamps_target[0])
    )
    return poses_interpolated, valid_idxs


def get_long_axis_angles_from_covs(covs):
    angles = []
    for cov in covs:
        eigen_values, eigen_vectors = np.linalg.eig(np.linalg.inv(cov))

        idx = eigen_values.argsort()[::-1]
        eigen_values = eigen_values[idx]
        eigen_vectors = eigen_vectors[:, idx]
        angle = np.arctan2(eigen_vectors[0, 0], eigen_vectors[0, 1])
        angles.append(angle)
    return angles


def transform_errors(errors, angles):
    assert len(errors) == len(angles), "Length of errors and angles should be the same"
    assert errors.shape[1] == 2, "Dimension of errors should be 2 (x&y)"
    errors_transformed = []
    for error, angle in zip(errors, angles):
        mat = np.array([[np.cos(angle), np.sin(angle)], [-np.sin(angle), np.cos(angle)]])
        error_transformed = (mat @ error.reshape(-1, 1)).reshape(-1)
        errors_transformed.append(error_transformed)
    errors_transformed = np.array(errors_transformed)
    return errors_transformed


def calc_body_frame_length(cov_list, pose_list):
    inv_2d = np.linalg.inv(cov_list[:, :2, :2])
    stddev_longitudinal_2d = []
    stddev_lateral_2d = []

    for i in range(len(cov_list)):
        cov_matrix = inv_2d[i]
        yaw = pose_list[i, 4]
        stddev_longitudinal_2d.append(calc_stddev_rotated(cov_matrix, yaw - np.pi / 2))
        stddev_lateral_2d.append(calc_stddev_rotated(cov_matrix, yaw))

    stddev_longitudinal_2d = np.array(stddev_longitudinal_2d)
    stddev_lateral_2d = np.array(stddev_lateral_2d)
    return stddev_longitudinal_2d, stddev_lateral_2d


def calc_long_short_radius(cov_list):
    cov_eig_val_list = np.linalg.eig(cov_list[:, :2, :2])[0]
    cov_eig_val_list[cov_eig_val_list < 0] = 1e-5
    stddev_long_2d = np.sqrt(np.max(cov_eig_val_list, axis=1))
    stddev_short_2d = np.sqrt(np.min(cov_eig_val_list, axis=1))
    return stddev_long_2d, stddev_short_2d


def calc_roc_curve(threshold_error, error_results: ErrorResults):
    positives = error_results.error > threshold_error
    if not positives.any():
        return None
    recall_list = []
    threshold_stddev_list = np.arange(0, 0.8, 0.02)
    for threshold_stddev in threshold_stddev_list:
        true_positives = positives & (error_results.expected_error > threshold_stddev)
        recall = np.sum(true_positives) / np.sum(positives)
        recall_list.append([threshold_stddev, recall])
    return np.array(recall_list)


def get_duration_to_error(timestamps, ndt_timestamps, error_lateral, ndt_freq=10):
    duration_to_error = []
    for timestamp, error in zip(timestamps, error_lateral):
        idx = bisect.bisect_left(ndt_timestamps, timestamp)
        if idx > 0 and error < 2:
            duration = timestamp - ndt_timestamps[idx - 1]
            if duration > 5.0 / ndt_freq:  # Only count if NDT hasn't come for 5 steps.
                duration_to_error.append([duration, error])
    return np.array(duration_to_error)


if __name__ == "__main__":
    # Apply some test here as a temporary measure.
    # Ideally this test should be implemented as a rostest.

    # get_long_axis_angles_from_covs
    cov0 = np.array([[1, 0], [0, 2]])
    cov1 = np.array([[2, 0], [0, 1]])
    cov2 = np.linalg.inv(np.array([[2, -1], [-1, 2]]))
    covs = [cov0, cov1, cov2]
    angles_answer = [90, 0, 45]
    np.testing.assert_array_almost_equal(
        get_long_axis_angles_from_covs(covs), np.deg2rad(angles_answer)
    )

    # transform_errors
    error = np.array([2, 0])
    errors = np.array([error, error, error, error])
    angles = np.deg2rad([0, 45, 90, 180])
    errors_transformed_answer = np.array([[2, 0], [np.sqrt(2), -np.sqrt(2)], [0, -2], [-2, 0]])
    errors_transformed_calculated = transform_errors(errors, angles)
    np.testing.assert_array_almost_equal(errors_transformed_calculated, errors_transformed_answer)
