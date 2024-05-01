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
import logging
import threading
import time
from typing import Dict
from typing import List

from PySide2.QtCore import QObject
from PySide2.QtCore import QTimer
from PySide2.QtCore import Signal
from geometry_msgs.msg import Transform
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import FramePair
from tier4_calibration_msgs.msg import CalibrationResult
from tier4_calibration_msgs.srv import ExtrinsicCalibrator


class CalibratorServiceWrapper(QObject):
    data_changed = Signal()
    status_signal = Signal(bool)
    status_changed_signal = Signal()
    result_signal = Signal(list)

    def __init__(
        self,
        ros_interface: RosInterface,
        service_name: str,
        expected_calibration_frames: List[FramePair],
    ):
        super().__init__()

        self.ros_interface = ros_interface
        self.service_name = service_name
        self.expected_calibration_frames = expected_calibration_frames

        self.calibration_results = defaultdict(lambda: defaultdict(Transform))
        self.service_status = False
        self.service_called = False

        self.children_to_id = {}
        self.parents = []
        self.children = []
        self.scores = []
        self.status_messages = []
        self.finished_list = []
        self.elapsed_times = []
        self.start_times = []

        for i, (parent_frame, child_frame) in enumerate(expected_calibration_frames):
            self.children_to_id[child_frame] = i
            self.parents.append(parent_frame)
            self.children.append(child_frame)
            self.scores.append("")
            self.status_messages.append("")
            self.finished_list.append(False)
            self.scores.append("")
            self.elapsed_times.append("")
            self.start_times.append(0)

        ros_interface.register_calibration_service(
            self.service_name, self.result_ros_callback, self.status_ros_callback
        )
        self.result_signal.connect(self.on_result)
        self.status_signal.connect(self.on_status)

        self.timer = QTimer()
        self.timer.timeout.connect(self.on_timer)

        # Create the services with a lock on the interface
        # register a callback and use the signal trick to execute in the UI thread
        # use partials

        # register the status checkers

    def start(self):
        self.ros_interface.call_calibration_service(self.service_name)

        t0 = time.time()
        self.start_times = [t0 for _ in range(len(self.parents))]
        self.status_messages = [
            "Service called. Waiting for calibration to end" for _ in range(len(self.parents))
        ]
        self.service_called = True

        self.data_changed.emit()
        self.timer.start(100)

    def on_timer(self):
        tf = time.time()

        for i in range(len(self.elapsed_times)):
            self.elapsed_times[i] = f"{tf - self.start_times[i]:.2f}"  # noqa E231

        self.data_changed.emit()

    def on_result(self, calibration_results: List[CalibrationResult]):
        received_calibration_ids = []
        tf = time.time()

        for calibration_result in calibration_results:
            parent_frame = calibration_result.transform_stamped.header.frame_id
            child_frame = calibration_result.transform_stamped.child_frame_id

            if parent_frame not in self.parents or child_frame not in self.children:
                logging.error(
                    f"Calibration result {parent_frame} -> {child_frame} was not expected. This is probably a configuration error in the launchers"
                )
                continue

            i = self.children_to_id[child_frame]

            if self.parents[i] != parent_frame:
                logging.error(
                    f"Calibration result {parent_frame} -> {child_frame} was not expected. This is probably a configuration error in the launchers"
                )
                continue

            self.calibration_results[parent_frame][
                child_frame
            ] = calibration_result.transform_stamped.transform

            self.scores[i] = calibration_result.score
            self.status_messages[i] = calibration_result.message.data
            self.finished_list[i] = True
            self.elapsed_times[i] = f"{tf - self.start_times[i]:.2f}"  # noqa E231
            received_calibration_ids.append(i)

        for i in range(len(self.parents)):
            if i not in received_calibration_ids:
                self.status_messages[i] = "Error. tf not included in the result"

        self.timer.stop()
        self.data_changed.emit()

    def on_status(self, status: bool):
        if self.service_status != status:
            self.service_status = status
            self.data_changed.emit()
            self.status_changed_signal.emit()

    def is_available(self) -> bool:
        return self.service_status

    def finished(self):
        return all(self.finished_list)

    def result_ros_callback(self, result: ExtrinsicCalibrator.Response):
        logging.debug(f"{threading.get_ident() }: result_ros_callback")
        self.result_signal.emit(result.results)

    def status_ros_callback(self, status: bool):
        self.status_signal.emit(status)

    def get_data(self, index) -> list:
        if not self.service_called:
            status_message = "Service ready" if self.service_status else "Service not available"
        else:
            status_message = self.status_messages[index]

        return [
            self.service_name,
            self.parents[index],
            self.children[index],
            self.elapsed_times[index],
            self.scores[index],
            status_message,
        ]

    def get_number_of_frames(self) -> int:
        return len(self.parents)

    def get_calibration_results(self) -> Dict[str, Dict[str, Transform]]:
        return self.calibration_results
