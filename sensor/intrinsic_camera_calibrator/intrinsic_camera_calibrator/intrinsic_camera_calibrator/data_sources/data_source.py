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


from enum import Enum
import threading


class DataSourceEnum(Enum):
    TOPIC = "ROS topic"
    BAG2 = "ROS bag"
    VIDEO = "Video file"
    FILES = "Image files"

    def __str__(self):
        return str(self.value)


class DataSource:
    def __init__(self, **kwargs):
        self.lock = threading.RLock()
        self.data_callback = None
        self.pending_data_not_consumed = False
        self.camera_name = "camera"
        pass

    def set_data_callback(self, callback):
        """Set a callback method for the DataSource to call when an image is produced."""
        self.data_callback = callback
        self.pending_data_not_consumed = True

    def consumed(self):
        """Notify the DataSource that that the data has been consumed."""
        with self.lock:
            self.pending_data_not_consumed = False

    def get_camera_name(self) -> str:
        return self.camera_name
