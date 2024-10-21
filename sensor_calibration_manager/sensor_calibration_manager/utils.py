#!/usr/bin/env python3

# Copyright 2024 TIER IV, Inc.
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

from geometry_msgs.msg import Transform
import numpy as np
import transforms3d


def tf_message_to_transform_matrix(msg):
    transform_matrix = np.eye(4)

    q = msg.rotation
    rotation_matrix = transforms3d.quaternions.quat2mat((q.w, q.x, q.y, q.z))

    transform_matrix[0:3, 0:3] = rotation_matrix
    transform_matrix[0, 3] = msg.translation.x
    transform_matrix[1, 3] = msg.translation.y
    transform_matrix[2, 3] = msg.translation.z

    return transform_matrix


def transform_matrix_to_tf_message(transform_matrix):
    q = transforms3d.quaternions.mat2quat(transform_matrix[0:3, 0:3])

    msg = Transform()
    msg.translation.x = transform_matrix[0, 3]
    msg.translation.y = transform_matrix[1, 3]
    msg.translation.z = transform_matrix[2, 3]
    msg.rotation.x = q[1]
    msg.rotation.y = q[2]
    msg.rotation.z = q[3]
    msg.rotation.w = q[0]

    return msg


def decompose_transformation_matrix(transformation):
    return transformation[0:3, 3].reshape(3, 1), transformation[0:3, 0:3]


def stamp_to_seconds(time):
    return time.sec + 1e-9 * time.nanosec
