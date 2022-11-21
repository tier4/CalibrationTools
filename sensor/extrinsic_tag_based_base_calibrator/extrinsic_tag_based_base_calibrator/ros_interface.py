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

import threading

import rclpy

# from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import Empty
from tier4_calibration_msgs.srv import Files


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


class FilesServiceWrapper(ServiceWrapper):
    def __init__(self, node, name):
        super().__init__()
        self.client = node.create_client(Files, name)

    def __call__(self, files):
        req = Files.Request()
        req.files = files
        self.future = self.client.call_async(req)


class RosInterface(Node):
    def __init__(self):

        super().__init__("extrinsic_tag_based_base_calibrator")

        try:
            self.declare_parameter("calibration_sensor_type", rclpy.Parameter.Type.STRING)

            self.calibration_sensor_type = (
                self.get_parameter("calibration_sensor_type").get_parameter_value().string_value
            )
        except Exception as e:
            print(e)
            self.calibration_sensor_type = "camera"

        self.ros_context = None
        self.ros_executor = None
        self.lock = threading.RLock()

        # Scene building related clients
        self.add_scene_client = EmptyServiceWrapper(self, "add_scene")
        self.add_external_camera_images_to_scene_client = FilesServiceWrapper(
            self, "add_external_camera_images_to_scene"
        )

        self.add_calibration_camera_images_to_scene_client = FilesServiceWrapper(
            self, "add_calibration_camera_images_to_scene"
        )
        self.add_calibration_camera_detections_to_scene_client = EmptyServiceWrapper(
            self, "add_calibration_camera_detections_to_scene"
        )
        self.add_calibration_lidar_detections_to_scene_client = EmptyServiceWrapper(
            self, "add_calibration_lidar_detections_to_scene"
        )

        # External camera intrinsics related clients
        self.load_external_camera_intrinsics_client = FilesServiceWrapper(
            self, "load_external_camera_intrinsics"
        )
        self.save_external_camera_intrinsics_client = FilesServiceWrapper(
            self, "save_external_camera_intrinsics"
        )
        self.calibrate_external_camera_intrinsics_client = FilesServiceWrapper(
            self, "calibrate_external_camera_intrinsics"
        )

        # Calibration camera intrinsics related clients
        self.load_calibration_camera_intrinsics_client = FilesServiceWrapper(
            self, "load_calibration_camera_intrinsics"
        )
        self.save_calibration_camera_intrinsics_client = FilesServiceWrapper(
            self, "save_calibration_camera_intrinsics"
        )
        self.calibrate_calibration_camera_intrinsics_client = FilesServiceWrapper(
            self, "calibrate_calibration_camera_intrinsics"
        )

        # Calibration related clients
        self.process_scenes_client = EmptyServiceWrapper(self, "process_scenes")
        self.calibration_client = EmptyServiceWrapper(self, "calibrate")

        # Calibration DB related clients
        self.load_database_client = FilesServiceWrapper(self, "load_database")
        self.save_database_client = FilesServiceWrapper(self, "save_database")

        self.client_list = [
            self.add_scene_client,
            self.add_external_camera_images_to_scene_client,
            self.add_calibration_camera_images_to_scene_client,
            self.add_calibration_camera_detections_to_scene_client,
            self.add_calibration_lidar_detections_to_scene_client,
            self.load_external_camera_intrinsics_client,
            self.save_external_camera_intrinsics_client,
            self.calibrate_external_camera_intrinsics_client,
            self.load_calibration_camera_intrinsics_client,
            self.save_calibration_camera_intrinsics_client,
            self.calibrate_calibration_camera_intrinsics_client,
            self.process_scenes_client,
            self.calibration_client,
            self.load_database_client,
            self.save_database_client,
        ]

        self.timer = self.create_timer(0.1, self.timer_callback)

    def get_sensor_type(self):
        return self.calibration_sensor_type

    def set_add_scene_callback(self, result_callback, status_callback):
        with self.lock:
            self.add_scene_client.set_result_callback(result_callback)
            self.add_scene_client.set_status_callback(status_callback)

    def set_add_external_camera_images_to_scene_callback(self, result_callback, status_callback):
        with self.lock:
            self.add_external_camera_images_to_scene_client.set_result_callback(result_callback)
            self.add_external_camera_images_to_scene_client.set_status_callback(status_callback)

    def set_add_calibration_camera_images_to_scene_callback(self, result_callback, status_callback):
        with self.lock:
            self.add_calibration_camera_images_to_scene_client.set_result_callback(result_callback)
            self.add_calibration_camera_images_to_scene_client.set_status_callback(status_callback)

    def set_add_calibration_camera_detections_to_scene_callback(
        self, result_callback, status_callback
    ):
        with self.lock:
            self.add_calibration_camera_detections_to_scene_client.set_result_callback(
                result_callback
            )
            self.add_calibration_camera_detections_to_scene_client.set_status_callback(
                status_callback
            )

    def set_add_calibration_lidar_detections_to_scene_callback(
        self, result_callback, status_callback
    ):
        with self.lock:
            self.add_calibration_lidar_detections_to_scene_client.set_result_callback(
                result_callback
            )
            self.add_calibration_lidar_detections_to_scene_client.set_status_callback(
                status_callback
            )

    def set_load_external_camera_intrinsics_callback(self, result_callback, status_callback):
        with self.lock:
            self.load_external_camera_intrinsics_client.set_result_callback(result_callback)
            self.load_external_camera_intrinsics_client.set_status_callback(status_callback)

    def set_save_external_camera_intrinsics_callback(self, result_callback, status_callback):
        with self.lock:
            self.save_external_camera_intrinsics_client.set_result_callback(result_callback)
            self.save_external_camera_intrinsics_client.set_status_callback(status_callback)

    def set_calibrate_external_camera_intrinsics_callback(self, result_callback, status_callback):
        with self.lock:
            self.calibrate_external_camera_intrinsics_client.set_result_callback(result_callback)
            self.calibrate_external_camera_intrinsics_client.set_status_callback(status_callback)

    def set_load_calibration_camera_intrinsics_callback(self, result_callback, status_callback):
        with self.lock:
            self.load_calibration_camera_intrinsics_client.set_result_callback(result_callback)
            self.load_calibration_camera_intrinsics_client.set_status_callback(status_callback)

    def set_save_calibration_camera_intrinsics_callback(self, result_callback, status_callback):
        with self.lock:
            self.save_calibration_camera_intrinsics_client.set_result_callback(result_callback)
            self.save_calibration_camera_intrinsics_client.set_status_callback(status_callback)

    def set_calibrate_calibration_camera_intrinsics_callback(
        self, result_callback, status_callback
    ):
        with self.lock:
            self.calibrate_calibration_camera_intrinsics_client.set_result_callback(result_callback)
            self.calibrate_calibration_camera_intrinsics_client.set_status_callback(status_callback)

    def set_process_scenes_callback(self, result_callback, status_callback):
        with self.lock:
            self.process_scenes_client.set_result_callback(result_callback)
            self.process_scenes_client.set_status_callback(status_callback)

    def set_calibration_callback(self, result_callback, status_callback):
        with self.lock:
            self.calibration_client.set_result_callback(result_callback)
            self.calibration_client.set_status_callback(status_callback)

    def set_load_database_callback(self, result_callback, status_callback):
        with self.lock:
            self.load_database_client.set_result_callback(result_callback)
            self.load_database_client.set_status_callback(status_callback)

    def set_save_database_callback(self, result_callback, status_callback):
        with self.lock:
            self.save_database_client.set_result_callback(result_callback)
            self.save_database_client.set_status_callback(status_callback)

    def add_scene(self):
        self.add_scene_client()

    def add_external_camera_images_to_scene(self, files):
        self.add_external_camera_images_to_scene_client(files)

    def add_calibration_camera_images_to_scene(self, files):
        self.add_calibration_camera_images_to_scene_client(files)

    def add_calibration_camera_detections_to_scene(self):
        self.add_calibration_camera_detections_to_scene_client()

    def add_calibration_lidar_detections_to_scene(self):
        self.add_calibration_lidar_detections_to_scene_client()

    def load_external_camera_intrinsics(self, files):
        self.load_external_camera_intrinsics_client(files)

    def save_external_camera_intrinsics(self, files):
        self.save_external_camera_intrinsics_client(files)

    def calibrate_external_camera_intrinsics(self, files):
        self.calibrate_external_camera_intrinsics_client(files)

    def load_calibration_camera_intrinsics(self, files):
        self.load_calibration_camera_intrinsics_client(files)

    def save_calibration_camera_intrinsics(self, files):
        self.save_calibration_camera_intrinsics_client(files)

    def calibrate_calibration_camera_intrinsics(self, files):
        self.calibrate_calibration_camera_intrinsics_client(files)

    def process_scenes(self):
        self.process_scenes_client()

    def calibrate(self):
        self.calibration_client()

    def load_database(self, files):
        self.load_database_client(files)

    def save_database(self, files):
        self.save_database_client(files)

    def timer_callback(self):

        with self.lock:

            for client in self.client_list:
                client.update()

    def spin(self):

        self.ros_executor = MultiThreadedExecutor(num_threads=2)
        self.ros_executor.add_node(self)

        self.thread = threading.Thread(target=self.executor.spin, args=())
        self.thread.setDaemon(True)
        self.thread.start()
