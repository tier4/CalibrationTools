import threading

import cv2
from cv_bridge import CvBridge
from intrinsic_camera_calibrator.data_sources.data_source import DataSource
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image


class RosTopicDataSource(DataSource, Node):
    def __init__(self):
        DataSource.__init__(self)
        Node.__init__(self, "intrinsic_camera_calibrator")

        self.ros_executor = None
        self.bridge = CvBridge()
        self.spin()

    def get_filtered_image_topics_and_types(self):

        topics_list = self.get_topic_names_and_types()
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

    def get_image_topics(self):

        filtered_topic_names_and_types = self.get_filtered_image_topics_and_types()

        return [topic_name for topic_name, _ in filtered_topic_names_and_types]

    def set_image_topic(self, image_topic):

        topics = self.get_filtered_image_topics_and_types()
        topics_dict = dict(topics)

        if image_topic not in topics_dict:
            return False

        for image_type in topics_dict[image_topic]:

            if image_type == "sensor_msgs/msg/CompressedImage":
                self.compressed_image_sub = self.create_subscription(
                    CompressedImage,
                    image_topic,
                    self.compressed_image_callback,
                    qos_profile_sensor_data,
                )
            else:
                self.image_sub = self.create_subscription(
                    Image, image_topic, self.image_callback, qos_profile_sensor_data
                )

    def compressed_image_callback(self, msg):
        with self.lock:
            image_data = np.frombuffer(msg.data, np.uint8)
            image_data = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
            self.data_callback(image_data)

    def image_callback(self, msg):
        with self.lock:
            image_data = self.bridge.imgmsg_to_cv2(self.image_sync)
            self.data_callback(image_data)

    def spin(self):

        self.thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.thread.setDaemon(True)
        self.thread.start()
