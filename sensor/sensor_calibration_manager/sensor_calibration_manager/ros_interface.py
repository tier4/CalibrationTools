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

from collections import defaultdict
import threading

import rclpy
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tier4_calibration_msgs.srv import ExtrinsicCalibrator


class RosInterface(Node):
    def __init__(self):
        super().__init__("sensor_calibration_manager")

        self.lock = threading.RLock()
        self.ros_executor = None

        # ROS Interface configuration
        self.publish_tf = None

        # UI interface.
        self.tfs_available_ui_callback = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.tf_qos_profile = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=100,
        )

        self.tf_sub = self.create_subscription(
            TFMessage,
            "/tf_static",
            self.tf_callback,
            self.tf_qos_profile,
        )

        self.tf_graph_available = False
        self.tf_msg = defaultdict(lambda: defaultdict(list))

        self.calibration_services_dict = {}
        self.calibration_futures_dict = {}
        self.calibration_result_callback_dict = {}
        self.calibration_status_callback_dict = {}
        self.calibration_service_start_dict = {}

        self.timer = self.create_timer(1.0, self.timer_callback)

    def set_tfs_graph_callback(self, callback):
        self.tfs_graph_ui_callback = callback

    def tf_callback(self, msg):
        for transform in msg.transforms:
            self.tf_msg[transform.header.frame_id][transform.child_frame_id] = transform.transform

        self.tfs_graph_ui_callback(self.tf_msg)

    def can_transform(self, parent: str, child: str):
        return self.tf_buffer.can_transform(
            parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.0)
        )

    def get_transform(self, parent: str, child: str):
        assert self.tf_buffer.can_transform(
            parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.0)
        )
        with self.lock:
            return self.tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time(), timeout=Duration(seconds=0.0)
            ).transform

    def timer_callback(self):
        with self.lock:
            for service_name, client in self.calibration_services_dict.items():
                if self.calibration_service_start_dict[service_name]:
                    continue

                service_status = client.service_is_ready()
                self.calibration_status_callback_dict[service_name](service_status)

            for service_name in list(self.calibration_futures_dict.keys()):
                future = self.calibration_futures_dict[service_name]

                if future.done():
                    self.calibration_result_callback_dict[service_name](future.result())
                    self.calibration_futures_dict.pop(service_name)

    def register_calibration_service(self, service_name, result_callback, status_callback):
        with self.lock:
            self.calibration_services_dict[service_name] = self.create_client(
                ExtrinsicCalibrator, service_name
            )

            self.calibration_result_callback_dict[service_name] = result_callback
            self.calibration_status_callback_dict[service_name] = status_callback
            self.calibration_service_start_dict[service_name] = False

    def call_calibration_service(self, service_name):
        with self.lock:
            req = ExtrinsicCalibrator.Request()
            future = self.calibration_services_dict[service_name].call_async(req)
            self.calibration_futures_dict[service_name] = future
            self.calibration_service_start_dict[service_name] = True

            # We stop listening for tfs after calibration starts (depending on the calibrator, tfs may change and we do not want them to affect the behavior of this node)
            self.tf_listener.unregister()

    def spin(self):
        self.ros_executor = SingleThreadedExecutor()
        self.ros_executor.add_node(self)

        self.thread = threading.Thread(target=self.executor.spin, args=())
        self.thread.setDaemon(True)
        self.thread.start()
