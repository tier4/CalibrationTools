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

import cv2
from geometry_msgs.msg import TransformStamped
import numpy as np
import transforms3d


def tf_message_to_transform_matrix(msg):
    transform_matrix = np.eye(4)

    q = msg.transform.rotation
    rotation_matrix = transforms3d.quaternions.quat2mat((q.w, q.x, q.y, q.z))

    transform_matrix[0:3, 0:3] = rotation_matrix
    transform_matrix[0, 3] = msg.transform.translation.x
    transform_matrix[1, 3] = msg.transform.translation.y
    transform_matrix[2, 3] = msg.transform.translation.z

    return transform_matrix


def transform_matrix_to_tf_message(transform_matrix):
    q = transforms3d.quaternions.mat2quat(transform_matrix[0:3, 0:3])

    msg = TransformStamped()
    msg.transform.translation.x = transform_matrix[0, 3]
    msg.transform.translation.y = transform_matrix[1, 3]
    msg.transform.translation.z = transform_matrix[2, 3]
    msg.transform.rotation.x = q[1]
    msg.transform.rotation.y = q[2]
    msg.transform.rotation.z = q[3]
    msg.transform.rotation.w = q[0]

    return msg


def transform_matrix_to_cv(transform_matrix):
    rotation_matrix = transform_matrix[0:3, 0:3]
    rvec, _ = cv2.Rodrigues(rotation_matrix)
    tvec = transform_matrix[0:3, 3].reshape(3, 1)

    return tvec, rvec


def cv_to_transformation_matrix(tvec, rvec):
    transform_matrix = np.eye(4)

    rotation_matrix, _ = cv2.Rodrigues(rvec)

    transform_matrix[0:3, 0:3] = rotation_matrix
    transform_matrix[0:3, 3] = tvec.reshape(
        3,
    )

    return transform_matrix


def decompose_transformation_matrix(transformation):
    return transformation[0:3, 3].reshape(3, 1), transformation[0:3, 0:3]


def transform_points(translation_vector, rotation_matrix, point_array):
    _, dim = point_array.shape
    assert dim == 3

    return np.dot(point_array, np.transpose(rotation_matrix)) + translation_vector.reshape(1, 3)


def stamp_to_seconds(time):
    return time.sec + 1e-9 * time.nanosec
