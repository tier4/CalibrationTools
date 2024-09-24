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

import json
import os
import threading
import time

from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import qos_profile_system_default
from rosidl_runtime_py.convert import message_to_ordereddict
from tf2_ros import TransformException
from tier4_calibration_msgs.msg import CalibrationResult
from tier4_calibration_msgs.srv import ExtrinsicCalibrator
from tier4_calibration_views.image_view_ros_interface import ImageViewRosInterface
from tier4_calibration_views.utils import decompose_transformation_matrix
from tier4_calibration_views.utils import tf_message_to_transform_matrix
from tier4_calibration_views.utils import transform_matrix_to_tf_message
from tier4_calibration_views.utils import transform_points
import transforms3d


class InteractiveCalibratorRosInterface(ImageViewRosInterface):
    def __init__(self):
        super().__init__("interactive_calibrator")

        self.lock = threading.RLock()

        self.declare_parameter("use_calibration_api", True)
        self.declare_parameter("can_publish_tf", True)

        self.use_calibration_api = (
            self.get_parameter("use_calibration_api").get_parameter_value().bool_value
        )
        self.can_publish_tf = self.get_parameter("can_publish_tf").get_parameter_value().bool_value

        self.new_output_tf = False

        self.calibration_error = np.inf
        self.output_transform_msg = None
        self.calibration_api_sent_pending = False

        # ROS Interface configuration
        self.publish_tf = None
        self.object_point_callback = None
        self.calibration_api_request_received_callback = None
        self.calibration_api_request_sent_callback = None

        self.point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, qos_profile_system_default
        )

        if self.use_calibration_api:
            self.service_callback_group = MutuallyExclusiveCallbackGroup()
            self.calibration_api_service_server = self.create_service(
                ExtrinsicCalibrator,
                "extrinsic_calibration",
                self.calibration_api_service_callback,
                callback_group=self.service_callback_group,
            )

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def send_calibration_api_result(self, calibration_error):
        with self.lock:
            self.calibration_api_sent_pending = True
            self.calibration_error = calibration_error

    def calibration_api_service_callback(
        self, request: ExtrinsicCalibrator.Request, response: ExtrinsicCalibrator.Response
    ):
        # Notify the UI that a request was received
        self.calibration_api_request_received_callback()

        # Loop until the response arrives
        while rclpy.ok():
            with self.lock:
                if self.calibration_api_sent_pending:
                    break

            time.sleep(1.0)

        with self.lock:
            assert self.output_transform_msg is not None

            result = CalibrationResult()
            result.success = True
            result.score = self.calibration_error
            result.message = "The score corresponds to the reprojection error"
            result.transform_stamped = self.output_transform_msg
            result.transform_stamped.header.frame_id = self.image_frame
            result.transform_stamped.child_frame_id = self.lidar_frame

            response.results.append(result)

        self.calibration_api_request_sent_callback()

        return response

    def set_object_point_callback(self, callback):
        with self.lock:
            self.object_point_callback = callback

    def set_calibration_api_request_received_callback(self, callback):
        self.calibration_api_request_received_callback = callback

    def set_calibration_api_request_sent_callback(self, callback):
        self.calibration_api_request_sent_callback = callback

    def set_publish_tf(self, value):
        with self.lock:
            self.publish_tf = value

    def set_camera_lidar_transform(self, camera_optical_lidar_transform):
        with self.lock:
            self.output_transform_msg = transform_matrix_to_tf_message(
                camera_optical_lidar_transform
            )

            self.output_transform_msg.header.frame_id = self.image_frame
            self.output_transform_msg.child_frame_id = self.lidar_frame
            self.new_output_tf = True

    def save_calibration_tfs(self, output_dir):
        with self.lock:
            d = message_to_ordereddict(self.output_transform_msg)

            q = self.output_transform_msg.transform.rotation
            e = transforms3d.euler.quat2euler((q.w, q.x, q.y, q.z))

            d["roll"] = e[0]
            d["pitch"] = e[1]
            d["yaw"] = e[2]

            with open(os.path.join(output_dir, "tf.json"), "w") as f:
                f.write(json.dumps(d, indent=4, sort_keys=False))

    def point_callback(self, point: PointStamped):
        point_xyz = np.array([point.point.x, point.point.y, point.point.z]).reshape(1, 3)

        if point.header.frame_id != self.lidar_frame:
            try:
                lidar_to_point_frame_tf = self.tf_buffer.lookup_transform(
                    self.lidar_frame,
                    point.header.frame_id,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
                lidar_to_point_frame_transform = tf_message_to_transform_matrix(
                    lidar_to_point_frame_tf
                )
                translation, rotation = decompose_transformation_matrix(
                    lidar_to_point_frame_transform
                )

                point_xyz = transform_points(translation, rotation, point_xyz)
            except TransformException as ex:
                self.get_logger().error(
                    f"Could not transform {self.lidar_frame} to {point.header.frame_id}: {ex}"
                )
                return

        self.object_point_callback(point_xyz)

    def timer_callback(self):
        super().timer_callback()

        with self.lock:
            now = self.get_clock().now().to_msg()

            if self.publish_tf and self.new_output_tf and self.can_publish_tf:
                self.output_transform_msg.header.stamp = now
                self.tf_publisher.sendTransform(self.output_transform_msg)
                self.new_output_tf = False
