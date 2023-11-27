#!/usr/bin/env python3

# Copyright 2023 Tier IV, Inc.
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

from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty


class ServiceWrapper:
    def __init__(self):
        self.client = None
        self.future = None
        self.status_callback = None
        self.result_callback = None
        self.service_status = False

    def update(self):
        service_status = self.client.service_is_ready()
        if service_status != self.service_status and self.status_callback is not None:
            self.service_status = service_status
            self.status_callback(service_status)

        if self.future is not None and self.future.done() and self.result_callback is not None:
            response = self.future.result()
            self.result_callback(response)
            self.future = None

    def set_status_callback(self, callback):
        self.status_callback = callback

    def set_result_callback(self, callback):
        self.result_callback = callback


class EmptyServiceWrapper(ServiceWrapper):
    def __init__(self, node, name):
        super().__init__()
        self.client = node.create_client(Empty, name)

    def __call__(self):
        req = Empty.Request()
        self.future = self.client.call_async(req)


class RosInterface(Node):
    def __init__(self):
        super().__init__("extrinsic_reflector_based_calibrator_ui")

        self.ros_context = None
        self.ros_executor = None
        self.lock = threading.RLock()

        self.extract_background_model_client = EmptyServiceWrapper(self, "extract_background_model")
        self.add_lidar_radar_pair_client = EmptyServiceWrapper(self, "add_lidar_radar_pair")
        self.delete_lidar_radar_pair_client = EmptyServiceWrapper(self, "delete_lidar_radar_pair")
        self.send_calibration_client = EmptyServiceWrapper(self, "send_calibration")
  

        self.client_list = [
            self.extract_background_model_client,
            self.add_lidar_radar_pair_client,
            self.delete_lidar_radar_pair_client,
            self.send_calibration_client,
        ]

        self.timer = self.create_timer(0.1, self.timer_callback)

    def set_extract_background_model_callback(self, result_callback, status_callback):
        with self.lock:
            self.extract_background_model_client.set_result_callback(result_callback)
            self.extract_background_model_client.set_status_callback(status_callback)

    def set_add_lidar_radar_pair_callback(self, result_callback, status_callback):
        with self.lock:
            self.add_lidar_radar_pair_client.set_result_callback(result_callback)
            self.add_lidar_radar_pair_client.set_status_callback(status_callback)

    def set_delete_lidar_radar_pair_callback(self, result_callback, status_callback):
        with self.lock:
            self.delete_lidar_radar_pair_client.set_result_callback(result_callback)
            self.delete_lidar_radar_pair_client.set_status_callback(status_callback)

    def set_send_calibration_callback(self, result_callback, status_callback):
        with self.lock:
            self.send_calibration_client.set_result_callback(result_callback)
            self.send_calibration_client.set_status_callback(status_callback)

    def extract_background_model(self):
        self.extract_background_model_client()

    def add_lidar_radar_pair(self):
        self.add_lidar_radar_pair_client()

    def send_calibration(self):
        self.send_calibration_client()

    def timer_callback(self):
        with self.lock:
            for client in self.client_list:
                client.update()

    def spin(self):
        self.ros_executor = SingleThreadedExecutor()
        self.ros_executor.add_node(self)

        self.thread = threading.Thread(target=self.executor.spin, args=())
        self.thread.setDaemon(True)
        self.thread.start()
