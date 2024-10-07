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

import array
from collections import deque
import threading
from typing import Callable
from typing import Deque
from typing import List
from typing import Optional
from typing import Union

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default
import ros2_numpy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from sensor_msgs.msg import PointCloud2
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tier4_calibration_msgs.msg import CalibrationPoints
from tier4_calibration_views.utils import stamp_to_seconds
from tier4_calibration_views.utils import tf_message_to_transform_matrix


class ImageViewRosInterface(Node):
    def __init__(self, node_name="image_view"):
        super().__init__(node_name)

        self.lock = threading.RLock()

        self.declare_parameter("use_rectified", False)
        self.declare_parameter("use_compressed", True)
        self.declare_parameter("timer_period", 1.0)
        self.declare_parameter("delay_tolerance", 0.06)

        self.use_rectified = self.get_parameter("use_rectified").get_parameter_value().bool_value
        self.use_compressed = self.get_parameter("use_compressed").get_parameter_value().bool_value
        self.timer_period = self.get_parameter("timer_period").get_parameter_value().double_value
        self.delay_tolerance = (
            self.get_parameter("delay_tolerance").get_parameter_value().double_value
        )

        self.image_frame: Optional[str] = None
        self.lidar_frame: Optional[str] = None

        # Data
        self.pointcloud_queue: Deque[PointCloud2] = deque([], 5)
        self.image_queue: Deque[Union[CompressedImage, Image]] = deque([], 5)

        self.camera_info: Optional[CameraInfo] = None
        self.pointcloud_sync: Optional[PointCloud2] = None
        self.image_sync: Optional[Union[CompressedImage, Image]] = None

        # ROS Interface configuration
        self.sensor_data_callback: Optional[Callable] = None
        self.sensor_data_delay_callback: Optional[Callable] = None
        self.transform_callback: Optional[Callable] = None
        self.external_calibration_points_callback: Optional[Callable] = None

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
            CalibrationPoints,
            "calibration_points_input",
            self.calibration_points_callback,
            qos_profile_system_default,
        )

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def set_sensor_data_callback(self, callback):
        with self.lock:
            self.sensor_data_callback = callback

    def set_sensor_data_delay_callback(self, callback):
        with self.lock:
            self.sensor_data_delay_callback = callback

    def set_transform_callback(self, callback):
        with self.lock:
            self.transform_callback = callback

    def set_external_calibration_points_callback(self, callback):
        with self.lock:
            self.external_calibration_points_callback = callback

    def pointcloud_callback(self, pointcloud_msg: PointCloud2):
        self.lidar_frame = pointcloud_msg.header.frame_id
        self.pointcloud_queue.append(pointcloud_msg)
        self.check_sync()

    def image_callback(self, image_msg: Union[CompressedImage, Image]):
        self.image_queue.append(image_msg)
        self.check_sync()

    def camera_info_callback(self, camera_info_msg: CameraInfo):
        self.camera_info = camera_info_msg
        self.image_frame = camera_info_msg.header.frame_id

        if self.use_rectified:

            self.camera_info.k[0] = self.camera_info.p[0]
            self.camera_info.k[2] = self.camera_info.p[2]
            self.camera_info.k[4] = self.camera_info.p[5]
            self.camera_info.k[5] = self.camera_info.p[6]
            self.camera_info.d = array.array("d", 0.0 * np.array(self.camera_info.d))

    def check_sync(self):
        if (
            len(self.image_queue) == 0
            or len(self.pointcloud_queue) == 0
            or self.camera_info is None
        ):
            return

        min_delay = np.inf

        for pointcloud_msg in self.pointcloud_queue:
            for image_msg in self.image_queue:
                current_delay = abs(
                    stamp_to_seconds(pointcloud_msg.header.stamp)
                    - stamp_to_seconds(image_msg.header.stamp)
                )

                min_delay = min(min_delay, current_delay)

                if min_delay <= self.delay_tolerance:
                    break

        if min_delay > self.delay_tolerance:
            self.sensor_data_delay_callback(min_delay)
            return

        pc_data = ros2_numpy.numpify(pointcloud_msg)
        points_np = np.zeros(pc_data.shape + (4,))
        points_np[..., 0] = pc_data["x"]
        points_np[..., 1] = pc_data["y"]
        points_np[..., 2] = pc_data["z"]
        points_np[..., 3] = (
            pc_data["intensity"]
            if "intensity" in pc_data.dtype.names
            else np.zeros_like(pc_data["x"])
        )
        points_np = points_np.reshape(-1, 4)

        with self.lock:
            self.camera_info_sync = self.camera_info
            self.image_sync = image_msg
            self.pointcloud_sync = pointcloud_msg

            if self.use_compressed:
                image_data = np.frombuffer(self.image_sync.data, np.uint8)
                self.image_sync = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            else:
                self.image_sync = self.bridge.imgmsg_to_cv2(self.image_sync, "bgr8")
                # image = cv2.cvtColor(self.raw_image, cv2.COLOR_BGR2RGB)

            self.sensor_data_callback(self.image_sync, self.camera_info_sync, points_np, min_delay)

        self.image_queue.clear()
        self.pointcloud_queue.clear()

    def calibration_points_callback(self, calibration_points: CalibrationPoints):
        object_points: List[Point] = calibration_points.object_points
        image_points: List[Point] = calibration_points.image_points

        assert len(object_points) == len(object_points)

        object_points = [np.array([p.x, p.y, p.z]) for p in object_points]
        image_points = [np.array([p.x, p.y]) for p in image_points]

        self.external_calibration_points_callback(object_points, image_points)

    def timer_callback(self):
        with self.lock:
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

    def spin(self):
        self.ros_executor = MultiThreadedExecutor(num_threads=2)
        self.ros_executor.add_node(self)

        self.thread = threading.Thread(target=self.executor.spin, args=())
        self.thread.setDaemon(True)
        self.thread.start()
