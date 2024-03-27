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


from functools import partial
import logging

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QCheckBox
from PySide2.QtWidgets import QDoubleSpinBox
from PySide2.QtWidgets import QGridLayout
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QSpinBox
from PySide2.QtWidgets import QWidget
from intrinsic_camera_calibrator.parameter import ParameterizedClass


class ParameterView(QWidget):
    """A simple widget to visualize and edit a ParameterizedClass's parameters."""

    parameter_changed = Signal()
    closed = Signal()

    def __init__(self, parameterized_class: ParameterizedClass):
        super().__init__()

        self.setWindowTitle("Edit parameters")

        self.layout = QGridLayout()
        self.setLayout(self.layout)

        self.parameterized_class = parameterized_class

        for i, (k, v) in enumerate(self.parameterized_class.parameters().items()):
            label = QLabel(k)
            self.layout.addWidget(label, i, 0)

            def on_value_changed(new_k, new_v):
                logging.info(f"on_value_changed {new_k}={new_v}")
                self.parameterized_class.set_parameters(**{new_k: new_v})
                self.parameter_changed.emit()

            if isinstance(v.value, bool):
                checkbox = QCheckBox()
                checkbox.setChecked(v.value)
                checkbox.stateChanged.connect(
                    partial(lambda k1, v1: on_value_changed(k1, v1 != 0), k)
                )
                self.layout.addWidget(checkbox, i, 1)

            elif isinstance(v.value, int):
                spinbox = QSpinBox()
                spinbox.setRange(v.min_value, v.max_value)
                spinbox.setSingleStep(1)
                spinbox.setValue(v.value)
                spinbox.valueChanged.connect(partial(on_value_changed, k))
                self.layout.addWidget(spinbox, i, 1)

            elif isinstance(v.value, float):
                spinbox = QDoubleSpinBox()
                spinbox.setDecimals(5)
                spinbox.setRange(v.min_value, v.max_value)
                spinbox.setSingleStep(abs(v.max_value - v.min_value) / 100.0)
                spinbox.setValue(v.value)
                spinbox.valueChanged.connect(partial(on_value_changed, k))
                self.layout.addWidget(spinbox, i, 1)
            else:
                raise NotImplementedError

        self.show()

    def closeEvent(self, event):
        """When the widget is closed it should be marked for deletion and notify the event."""
        self.closed.emit()
        event.accept()
        self.deleteLater()
