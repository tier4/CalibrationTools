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

from abc import ABCMeta
from abc import abstractproperty
from collections import defaultdict
import logging
from typing import Dict
from typing import List

from PySide2.QtCore import QObject
from PySide2.QtCore import QTimer
from PySide2.QtCore import Signal
from geometry_msgs.msg import Transform
import numpy as np
from sensor_calibration_manager.calibrator_wrapper import CalibratorServiceWrapper
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import CalibratorState
from sensor_calibration_manager.types import FramePair
from sensor_calibration_manager.utils import tf_message_to_transform_matrix
from sensor_calibration_manager.utils import transform_matrix_to_tf_message
import transforms3d
import yaml


class CalibratorBase(QObject):
    __metaclass__ = ABCMeta

    state_changed_signal = Signal(CalibratorState)
    calibration_finished_signal = Signal()

    def __init__(self, ros_interface: RosInterface):
        logging.debug("CalibratorBase: constructor start")
        super().__init__()
        self.ros_interface = ros_interface
        self.calibrators: List[CalibratorServiceWrapper] = []
        self.expected_calibration_frames: List[FramePair] = []
        self.state = CalibratorState.WAITING_TFS

        self.calibration_result_tfs = defaultdict(lambda: defaultdict(Transform))
        self.calibration_result_transforms = defaultdict(lambda: defaultdict(Transform))

        self.check_tf_timer = QTimer()
        self.check_tf_timer.timeout.connect(self.on_check_tf_timer)
        self.check_tf_timer.start(500)
        logging.debug("CalibratorBase: constructor end")

    def init():
        logging.debug("CalibratorBase: Calibrator init?")

    def on_check_tf_timer(self):
        logging.debug("CalibratorBase: on_check_tf_timer")
        assert self.state == CalibratorState.WAITING_TFS
        tfs_ready = all(
            self.ros_interface.can_transform(self.required_frames[0], frame)
            for frame in self.required_frames[1:]
        )

        if tfs_ready:
            self.state = CalibratorState.WAITING_SERVICES
            self.state_changed_signal.emit(self.state)
            self.check_tf_timer.stop()
            logging.debug("CalibratorBase: on_check_tf_timer stop")
        else:
            for frame in self.required_frames[1:]:
                if not self.ros_interface.can_transform(self.required_frames[0], frame):
                    logging.debug(f"could not transform {self.required_frames[0]} to {frame}")

    def get_transform_matrix(self, parent: str, child: str) -> np.array:
        if parent not in self.required_frames or child not in self.required_frames:
            raise ValueError
        tf_msg = self.ros_interface.get_transform(parent, child)
        return tf_message_to_transform_matrix(tf_msg)

    def add_calibrator(self, service_name: str, expected_calibration_frames: List[FramePair]):
        logging.debug("CalibratorBase: add_calibrator")

        for pair in expected_calibration_frames:
            assert (
                pair not in self.expected_calibration_frames
            ), f"The pair {pair} was already registered by a previous calibrator"
            self.expected_calibration_frames.append(pair)

        calibration_wrapper = CalibratorServiceWrapper(
            self.ros_interface, service_name, expected_calibration_frames
        )
        calibration_wrapper.status_changed_signal.connect(self.on_service_status_changed)
        calibration_wrapper.result_signal.connect(self.on_calibration_result)
        self.calibrators.append(calibration_wrapper)

    def on_service_status_changed(self):
        if self.state in [CalibratorState.WAITING_SERVICES, CalibratorState.READY]:
            services_available = all([calibrator.is_available() for calibrator in self.calibrators])

            if services_available and self.state == CalibratorState.WAITING_SERVICES:
                self.state = CalibratorState.READY
                self.state_changed_signal.emit(self.state)
            elif not services_available and self.state == CalibratorState.READY:
                self.state = CalibratorState.WAITING_SERVICES
                self.state_changed_signal.emit(self.state)

    def on_calibration_result(self):
        logging.debug("CalibratorBase: on_calibration_result")

        for calibrator in self.calibrators:
            d = calibrator.get_calibration_results()

            for parent_frame, d2 in d.items():
                for child_frame, transform in d2.items():
                    self.calibration_result_tfs[parent_frame][child_frame] = transform

            if not calibrator.finished():
                return

        self.post_process_internal()
        self.state = CalibratorState.FINISHED
        self.state_changed_signal.emit(self.state)
        self.calibration_finished_signal.emit()

    def get_service_wrappers(self) -> List[CalibratorServiceWrapper]:
        return self.calibrators

    def get_calibration_results(self) -> Dict[str, Dict[str, Transform]]:
        return self.calibration_result_tfs

    def get_processed_calibration_results(self) -> Dict[str, Dict[str, Transform]]:
        return self.processed_calibration_result_tfs

    def start_calibration(self):
        assert self.state == CalibratorState.READY

        self.pre_process()

        for calibrator in self.calibrators:
            calibrator.start()

        self.state = CalibratorState.CALIBRATING
        self.state_changed_signal.emit(self.state)

    @abstractproperty
    def required_frames(self) -> List[str]:
        raise NotImplementedError

    def pre_process(self):
        pass

    def post_process_internal(self):
        logging.debug("Before post_process")
        for parent, children_and_transforms in self.calibration_result_tfs.items():
            for child, transform in children_and_transforms.items():
                logging.debug(f"{parent}->{child}:\n{transform}")

        calibration_transforms = {
            parent: {
                child: tf_message_to_transform_matrix(transform)
                for child, transform in children_and_transforms.items()
            }
            for parent, children_and_transforms in self.calibration_result_tfs.items()
        }
        calibration_transforms = self.post_process(calibration_transforms)
        self.processed_calibration_result_tfs = {
            parent: {
                child: transform_matrix_to_tf_message(transform)
                for child, transform in children_and_transforms.items()
            }
            for parent, children_and_transforms in calibration_transforms.items()
        }

        logging.debug("After post_process")
        for parent, children_and_transforms in self.processed_calibration_result_tfs.items():
            for child, transform in children_and_transforms.items():
                logging.debug(f"{parent}->{child}:\n{transform}")

    def post_process(self, calibration_transforms) -> Dict[str, Dict[str, np.array]]:
        return calibration_transforms

    def save_results(self, path, use_rpy=True):
        output_dict = {}

        for parent, children_and_tfs_dict in self.get_processed_calibration_results().items():
            output_dict[parent] = {}

            for child, tf_msg in children_and_tfs_dict.items():
                translation = tf_msg.translation
                quat = tf_msg.rotation

                d = {}
                d["x"] = translation.x.item()
                d["y"] = translation.y.item()
                d["z"] = translation.z.item()

                if use_rpy:
                    roll, pitch, yaw = transforms3d.euler.quat2euler(
                        [quat.w, quat.x, quat.y, quat.z]
                    )
                    d["roll"] = roll
                    d["pitch"] = pitch
                    d["yaw"] = yaw
                else:
                    d["qx"] = quat.x.item()
                    d["qy"] = quat.y.item()
                    d["qz"] = quat.z.item()
                    d["qw"] = quat.w.item()

                output_dict[parent][child] = d

        def float_representer(dumper, value):
            text = "{0:.6f}".format(value)
            return dumper.represent_scalar("tag:yaml.org,2002:float", text)

        yaml.add_representer(float, float_representer)

        with open(path, "w") as f:
            yaml.dump(output_dict, f, sort_keys=False)
