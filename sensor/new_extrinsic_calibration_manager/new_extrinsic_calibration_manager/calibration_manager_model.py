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

import logging
from typing import List

from PySide2.QtCore import QAbstractTableModel
from PySide2.QtCore import Qt
from new_extrinsic_calibration_manager.calibrator_wrapper import CalibratorServiceWrapper


class CalibratorManagerModel(QAbstractTableModel):
    column_names = ["Service name", "Parent", "Child", "Elapsed time", "Score", "Status"]

    def __init__(self, calibrator_service_wrapper_list: List[CalibratorServiceWrapper]):
        super().__init__()
        self.calibrator_service_wrapper_list = calibrator_service_wrapper_list

        self.index_to_calibrator_index = []
        self.index_to_frame_index = []

        for calibrator_index, calibrator_wrapper in enumerate(self.calibrator_service_wrapper_list):
            for frame_index in range(calibrator_wrapper.get_number_of_frames()):
                self.index_to_calibrator_index.append(calibrator_index)
                self.index_to_frame_index.append(frame_index)

            calibrator_wrapper.data_changed.connect(self.on_changed)

    def on_changed(self):
        logging.debug("CalibratorManagerModel: on_changed")
        self.dataChanged.emit(
            self.createIndex(0, 0), self.createIndex(self.rowCount(0), self.columnCount(0))
        )

    def data(self, index, role):
        if role in [Qt.DisplayRole, Qt.ToolTipRole]:
            value = self.calibrator_service_wrapper_list[
                self.index_to_calibrator_index[index.row()]
            ].get_data(self.index_to_frame_index[index.row()])[index.column()]
            return str(value)

    def rowCount(self, index):
        return sum(
            [
                calibrator.get_number_of_frames()
                for calibrator in self.calibrator_service_wrapper_list
            ]
        )

    def columnCount(self, index):
        return len(self.column_names)

    def headerData(self, index, orientation, role):
        # section is the index of the column/row.
        if role == Qt.DisplayRole:
            if orientation == Qt.Horizontal:
                return str(self.column_names[index])

        super().headerData(index, orientation, role)
