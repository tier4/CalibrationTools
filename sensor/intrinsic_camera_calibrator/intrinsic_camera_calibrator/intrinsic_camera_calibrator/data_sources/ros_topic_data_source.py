#!/usr/bin/env python3

# Copyright 2022 Tier IV, Inc.
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

import threading
from typing import List
from typing import Tuple

import cv2
from cv_bridge import CvBridge
from intrinsic_camera_calibrator.data_sources.data_source import DataSource
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


class RosTopicDataSource(DataSource, Node):
    def __init__(self):
        DataSource.__init__(self)

        self.bridge = CvBridge()

        self.ros_context = rclpy.context.Context()

        rclpy.init(context=self.ros_context)
        Node.__init__(self, "intrinsic_camera_calibrator", context=self.ros_context)

        self.ros_executor = SingleThreadedExecutor(context=self.ros_context)
        self.ros_executor.add_node(self)

    def get_filtered_image_topics_and_types(self) -> List[Tuple[str, str]]:
        """Return a list of image topics and their types."""
        topics_list = self.get_topic_names_and_types()
        with self.lock:
            return [
                (topic_name, topic_types)
                for topic_name, topic_types in topics_list
                if len(
                    set(topic_types).intersection(
                        {"sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"}
                    )
                )
                > 0
            ]

    def get_image_topics(self) -> List[str]:
        """Return a list of the available topics in string format."""
        with self.lock:
            filtered_topic_names_and_types = self.get_filtered_image_topics_and_types()

            return [topic_name for topic_name, _ in filtered_topic_names_and_types]

    def set_image_topic(
        self,
        image_topic: str,
        reliability: rclpy.qos.ReliabilityPolicy,
        durability: rclpy.qos.DurabilityPolicy.VOLATILE,
    ):
        """Set the data source topic and subscribes to it."""
        with self.lock:

            topics = self.get_filtered_image_topics_and_types()
            topics_dict = dict(topics)

            if image_topic not in topics_dict:
                return False

            # Parse camera name
            topic_namespaces = image_topic.split("/")
            topic_namespaces.reverse()
            image_index = [i for i, s in enumerate(topic_namespaces) if "image" in s][0]
            self.camera_name = topic_namespaces[image_index + 1]

            self.qos_profile = rclpy.qos.QoSProfile(
                reliability=reliability,
                durability=durability,
                history=rclpy.qos.HistoryPolicy.SYSTEM_DEFAULT,
            )

            for image_type in topics_dict[image_topic]:

                if image_type == "sensor_msgs/msg/CompressedImage":
                    self.compressed_image_sub = self.create_subscription(
                        CompressedImage,
                        image_topic,
                        self.compressed_image_callback,
                        self.qos_profile,
                    )
                else:
                    self.image_sub = self.create_subscription(
                        Image, image_topic, self.image_callback, self.qos_profile
                    )

            self.spin()

    def compressed_image_callback(self, msg: CompressedImage):
        """Process a compressed image."""
        with self.lock:
            image_data = np.frombuffer(msg.data, np.uint8)
            image_data = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            self.data_callback(image_data)

    def image_callback(self, msg: Image):
        """Process a raw image."""
        with self.lock:
            image_data = self.bridge.imgmsg_to_cv2(msg)
            self.data_callback(image_data)

    def spin(self):
        """Start a new thread for ROS to spin in."""
        self.thread = threading.Thread(target=self.ros_executor.spin, args=())
        self.thread.setDaemon(True)
        self.thread.start()

    def stop(self):
        with self.lock:
            rclpy.shutdown(context=self.ros_context)
