#!/usr/bin/env python3

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

from collections import deque
import json
import os
import threading
import time

import cv2
from cv_bridge import CvBridge
from extrinsic_interactive_calibrator.utils import decompose_transformation_matrix
from extrinsic_interactive_calibrator.utils import stamp_to_seconds
from extrinsic_interactive_calibrator.utils import tf_message_to_transform_matrix
from extrinsic_interactive_calibrator.utils import transform_matrix_to_tf_message
from extrinsic_interactive_calibrator.utils import transform_points
from geometry_msgs.msg import Point
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
import ros2_numpy
from rosidl_runtime_py.convert import message_to_ordereddict
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tier4_calibration_msgs.msg import CalibrationPoints
from tier4_calibration_msgs.srv import ExtrinsicCalibrator
from tier4_calibration_msgs.srv import IntrinsicsOptimizer
import transforms3d
from visualization_msgs.msg import MarkerArray


class RosInterface(Node):
    def __init__(self):

        super().__init__("interactive_calibrator")

        self.lock = threading.RLock()

        self.declare_parameter("camera_parent_frame", rclpy.Parameter.Type.STRING)
        self.declare_parameter("camera_frame", rclpy.Parameter.Type.STRING)
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("delay_tolerance", 0.5)
        self.declare_parameter("use_calibration_api", True)
        self.declare_parameter("can_publish_tf", True)

        self.camera_parent_frame = (
            self.get_parameter("camera_parent_frame").get_parameter_value().string_value
        )
        self.camera_frame = self.get_parameter("camera_frame").get_parameter_value().string_value
        self.use_compressed = self.get_parameter("use_compressed").get_parameter_value().bool_value
        self.timer_period = (
            self.get_parameter("timer_period").get_parameter_value().double_value
        )  # 1.0
        self.delay_tolerance = (
            self.get_parameter("delay_tolerance").get_parameter_value().double_value
        )  # 0.03
        self.use_calibration_api = (
            self.get_parameter("use_calibration_api").get_parameter_value().bool_value
        )
        self.can_publish_tf = self.get_parameter("can_publish_tf").get_parameter_value().bool_value

        self.image_frame = None

        self.ros_context = None
        self.ros_executor = None

        # State
        self.paused = False

        # Data
        self.pointcloud_queue = deque([], 5)
        self.camera_info_queue = deque([], 5)
        self.image_queue = deque([], 5)

        self.pointcloud_sync = None
        self.camera_info_sync = None
        self.image_sync = None

        self.new_output_tf = False
        self.optimize_camera_intrinsics_available = False
        self.optimize_camera_intrinsics_future = None

        self.calibration_error = np.inf
        self.output_transform_msg = None
        self.calibration_api_sent_pending = False

        # ROS Interface configuration
        self.publish_tf = None
        self.republish_data = None
        self.sensor_data_callback = None
        self.transform_callback = None
        self.object_point_callback = None
        self.external_calibration_points_callback = None
        self.optimize_camera_intrinsics_status_callback = None
        self.optimize_camera_intrinsics_result_callback = None
        self.calibration_api_request_received_callback = None
        self.calibration_api_request_sent_callback = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_publisher = StaticTransformBroadcaster(self)
        self.bridge = CvBridge()

        self.lidar_subs = self.create_subscription(
            PointCloud2, "pointcloud", self.pointcloud_callback, qos_profile_sensor_data
        )

        if self.use_compressed:
            self.image_sub = self.create_subscription(
                CompressedImage, "image", self.image_callback, qos_profile_sensor_data
            )
        else:
            self.image_sub = self.create_subscription(
                Image, "image", self.image_callback, qos_profile_sensor_data
            )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "camera_info", self.camera_info_callback, qos_profile_sensor_data
        )
        self.point_sub = self.create_subscription(
            PointStamped, "/clicked_point", self.point_callback, qos_profile_system_default
        )
        self.point_sub = self.create_subscription(
            CalibrationPoints,
            "calibration_points_input",
            self.calibration_points_callback,
            qos_profile_system_default,
        )

        self.markers_pub = self.create_publisher(MarkerArray, "markers", qos_profile_sensor_data)
        self.image_pub = self.create_publisher(Image, "calibration/image", qos_profile_sensor_data)
        self.camera_info_pub = self.create_publisher(
            CameraInfo, "calibration/camera_info", qos_profile_sensor_data
        )
        self.pointcloud_pub = self.create_publisher(
            PointCloud2, "calibration/pointcloud", qos_profile_sensor_data
        )

        self.optimize_camera_intrinsics_client = self.create_client(
            IntrinsicsOptimizer, "optimize_intrinsics"
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
            self.calibration_error = -calibration_error

    def calibration_api_service_callback(self, request, response):

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

            response.success = True
            response.result_pose.position.x = self.output_transform_msg.transform.translation.x
            response.result_pose.position.y = self.output_transform_msg.transform.translation.y
            response.result_pose.position.z = self.output_transform_msg.transform.translation.z
            response.result_pose.orientation = self.output_transform_msg.transform.rotation

            response.score = self.calibration_error

        self.calibration_api_request_sent_callback()

        return response

    def set_sensor_data_callback(self, callback):
        with self.lock:
            self.sensor_data_callback = callback

    def set_transform_callback(self, callback):
        with self.lock:
            self.transform_callback = callback

    def set_object_point_callback(self, callback):
        with self.lock:
            self.object_point_callback = callback

    def set_external_calibration_points_callback(self, callback):
        with self.lock:
            self.external_calibration_points_callback = callback

    def set_optimize_camera_intrinsics_status_callback(self, callback):
        self.optimize_camera_intrinsics_status_callback = callback

    def set_optimize_camera_intrinsics_result_callback(self, callback):
        self.optimize_camera_intrinsics_result_callback = callback

    def set_calibration_api_request_received_callback(self, callback):
        self.calibration_api_request_received_callback = callback

    def set_calibration_api_request_sent_callback(self, callback):
        self.calibration_api_request_sent_callback = callback

    def set_republish_data(self, value):
        with self.lock:
            self.republish_data = value

    def set_publish_tf(self, value):
        with self.lock:
            self.publish_tf = value

    def is_paused(self):
        with self.lock:
            return self.paused

    def set_paused(self, value):
        with self.lock:
            self.paused = value

    def set_camera_lidar_transform(self, camera_optical_lidar_transform):
        with self.lock:

            optical_axis_to_camera_transform = np.zeros((4, 4))
            optical_axis_to_camera_transform[0, 1] = -1
            optical_axis_to_camera_transform[1, 2] = -1
            optical_axis_to_camera_transform[2, 0] = 1
            optical_axis_to_camera_transform[3, 3] = 1

            try:
                camera_parent_lidar_tf = self.tf_buffer.lookup_transform(
                    self.camera_parent_frame,
                    self.lidar_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=1.0),
                )
                camera_parent_lidar_transform = tf_message_to_transform_matrix(
                    camera_parent_lidar_tf
                )
            except TransformException as ex:
                self.get_logger().error(
                    f"Could not transform {self.camera_parent_frame} to {self.lidar_frame}: {ex}"
                )
                return

            camera_camera_parent_transform = (
                np.linalg.inv(optical_axis_to_camera_transform)
                @ camera_optical_lidar_transform
                @ np.linalg.inv(camera_parent_lidar_transform)
            )

            self.output_transform_msg = transform_matrix_to_tf_message(
                np.linalg.inv(camera_camera_parent_transform)
            )
            self.output_transform_msg.header.frame_id = self.camera_parent_frame
            self.output_transform_msg.child_frame_id = self.camera_frame
            self.new_output_tf = True

    def optimize_camera_intrinsics(self, object_points, image_points):

        req = IntrinsicsOptimizer.Request()

        for object_point, image_point in zip(object_points, image_points):

            point3d = Point()
            point3d.x = object_point[0]
            point3d.y = object_point[1]
            point3d.z = object_point[2]
            req.calibration_points.object_points.append(point3d)

            point2d = Point()
            point2d.x = image_point[0]
            point2d.y = image_point[1]
            req.calibration_points.image_points.append(point2d)

        req.initial_camera_info = self.camera_info_sync

        self.optimize_camera_intrinsics_future = self.optimize_camera_intrinsics_client.call_async(
            req
        )

    def save_calibration_tfs(self, output_dir):
        with self.lock:

            d = message_to_ordereddict(self.output_transform_msg)

            q = self.output_transform_msg.transform.rotation
            e = transforms3d.euler.quat2euler((q.w, q.x, q.y, q.z))

            d["roll"] = e[0]
            d["pitch"] = e[1]
            d["yaw"] = e[2]

            with open(os.path.join(output_dir, "tf.json"), "w") as fout:
                fout.write(json.dumps(d, indent=4, sort_keys=False))

    def pointcloud_callback(self, pointcloud_msg):

        self.lidar_frame = pointcloud_msg.header.frame_id
        self.pointcloud_queue.append(pointcloud_msg)

        t0 = time.time()
        self.check_sync()
        t1 = time.time()

    def image_callback(self, image_msg):

        self.image_queue.append(image_msg)
        self.check_sync()

    def camera_info_callback(self, camera_info_msg):

        self.camera_info_queue.append(camera_info_msg)
        self.image_frame = camera_info_msg.header.frame_id

    def check_sync(self):

        with self.lock:
            if self.paused:
                return

        if (
            len(self.camera_info_queue) == 0
            or len(self.image_queue) == 0
            or len(self.pointcloud_queue) == 0
        ):
            return

        found = False
        min_delay = 10000

        for pointcloud_msg in self.pointcloud_queue:
            for image_msg in self.image_queue:

                current_delay = abs(
                    stamp_to_seconds(pointcloud_msg.header.stamp)
                    - stamp_to_seconds(image_msg.header.stamp)
                )

                min_delay = min(min_delay, current_delay)

                if current_delay < self.delay_tolerance:
                    found = True
                    break

        if not found:
            return

        pc_data = ros2_numpy.numpify(pointcloud_msg)
        points = np.zeros(pc_data.shape + (4,))
        points[..., 0] = pc_data["x"]
        points[..., 1] = pc_data["y"]
        points[..., 2] = pc_data["z"]
        points[..., 3] = (
            pc_data["intensity"]
            if "intensity" in pc_data.dtype.names
            else np.zeros_like(pc_data["x"])
        )

        with self.lock:
            self.camera_info_sync = self.camera_info_queue[-1]
            self.image_sync = image_msg
            self.pointcloud_sync = pointcloud_msg

            if self.use_compressed:
                image_data = np.frombuffer(self.image_sync.data, np.uint8)
                self.image_sync = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            else:
                self.image_sync = self.bridge.imgmsg_to_cv2(self.image_sync)
                # image = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2RGB)

            self.sensor_data_callback(self.image_sync, self.camera_info_sync, points)

        self.image_queue.clear()  # this is suboptical but is only for the gui
        self.pointcloud_queue.clear()

    def point_callback(self, point):

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

    def calibration_points_callback(self, calibration_points):

        object_points = calibration_points.object_points
        image_points = calibration_points.image_points

        assert len(object_points) == len(object_points)

        object_points = [np.array([p.x, p.y, p.z]) for p in object_points]
        image_points = [np.array([p.x, p.y]) for p in image_points]

        self.external_calibration_points_callback(object_points, image_points)

    def timer_callback(self):

        with self.lock:

            service_status = self.optimize_camera_intrinsics_client.service_is_ready()
            if (
                service_status != self.optimize_camera_intrinsics_available
                and self.camera_info_sync is not None
            ):
                self.optimize_camera_intrinsics_status_callback(service_status)
                self.optimize_camera_intrinsics_available = service_status

            if (
                self.optimize_camera_intrinsics_future is not None
                and self.optimize_camera_intrinsics_future.done()
            ):
                response = self.optimize_camera_intrinsics_future.result()
                self.optimize_camera_intrinsics_result_callback(response.optimized_camera_info)
                self.optimize_camera_intrinsics_future = None

            if self.image_frame is None or self.lidar_frame is None:
                return

            try:
                transform = self.tf_buffer.lookup_transform(
                    self.image_frame,
                    self.lidar_frame,
                    rclpy.time.Time(seconds=0, nanoseconds=0),
                    timeout=Duration(seconds=0.2),
                )

                transform_matrix = tf_message_to_transform_matrix(transform)
                self.transform_callback(transform_matrix)
            except TransformException as ex:
                self.get_logger().error(
                    f"Could not transform {self.image_frame} to {self.lidar_frame}: {ex}"
                )

            now = self.get_clock().now().to_msg()

            if self.publish_tf and self.new_output_tf and self.can_publish_tf:
                self.output_transform_msg.header.stamp = now
                self.tf_publisher.sendTransform(self.output_transform_msg)
                self.new_output_tf = False

            if self.republish_data and self.image_sync is not None:
                assert self.camera_info_sync is not None
                assert self.pointcloud_sync is not None

                image_msg = self.bridge.cv2_to_imgmsg(self.image_sync)

                image_msg.header.stamp = now
                self.camera_info_sync.header.stamp = now
                self.pointcloud_sync.header.stamp = now

                self.image_pub.publish(image_msg)
                self.camera_info_pub.publish(self.camera_info_sync)
                self.pointcloud_pub.publish(self.pointcloud_sync)

    def spin(self):

        self.ros_executor = MultiThreadedExecutor(num_threads=2)
        self.ros_executor.add_node(self)

        self.thread = threading.Thread(target=self.executor.spin, args=())
        self.thread.setDaemon(True)
        self.thread.start()
