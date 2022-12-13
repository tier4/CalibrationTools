from functools import partial

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QCheckBox
from PySide2.QtWidgets import QDoubleSpinBox
from PySide2.QtWidgets import QGridLayout
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QSpinBox
from PySide2.QtWidgets import QWidget
from intrinsic_camera_calibrator.parameter import ParameteredClass


class ParameterView(QWidget):

    parameter_changed = Signal()
    closed = Signal()

    def __init__(self, parametered_class: ParameteredClass):
        super().__init__()

        self.setWindowTitle("Edit parameters")

        self.layout = QGridLayout()
        self.setLayout(self.layout)

        self.parametered_class = parametered_class

        for i, (k, v) in enumerate(self.parametered_class.parameters().items()):
            # print(f"{k}: {v}")
            label = QLabel(k)
            self.layout.addWidget(label, i, 0)

            def on_value_changed(new_k, new_v):
                print(f"on_value_changed {new_k}={new_v}")
                self.parametered_class.set_parameters(**{new_k: new_v})
                self.parameter_changed.emit()

            if isinstance(v.value, bool):
                checkbox = QCheckBox()
                checkbox.setChecked(v.value)
                checkbox.stateChanged.connect(partial(on_value_changed, k))
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
        print("Parameter view: closeEvent")
        self.closed.emit()
        event.accept()
        self.deleteLater()

    def __del__(self):
        print("Parameter view: destructor")
