from PySide2.QtCore import Signal
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QGroupBox
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QRadioButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from intrinsic_camera_calibrator.boards import BoardEnum
from intrinsic_camera_calibrator.data_sources.data_source import DataSource
from intrinsic_camera_calibrator.data_sources.data_source import DataSourceEnum
from intrinsic_camera_calibrator.data_sources.data_source_factory import make_data_source
from intrinsic_camera_calibrator.types import OperationMode
from intrinsic_camera_calibrator.views.parameter_view import ParameterView
from intrinsic_camera_calibrator.views.ros_topic_view import RosTopicView


class InitializationView(QWidget):

    parameter_changed = Signal()
    closed = Signal()

    def __init__(self, calibrator: "CameraIntrinsicsCalibratorUI", cfg):  # noqa F821
        super().__init__()

        self.setWindowTitle("Initial configuration")
        self.setMinimumWidth(300)

        self.calibrator = calibrator
        self.cfg = cfg
        self.data_source_view: QWidget = None
        self.data_source: DataSource = None

        self.layout = QVBoxLayout(self)

        # Source
        self.source_group = QGroupBox("Source options")
        self.source_group.setFlat(True)

        self.data_source_combobox = QComboBox()

        for data_source in DataSourceEnum:
            self.data_source_combobox.addItem(str(data_source), data_source)

        source_layout = QVBoxLayout()
        source_layout.addWidget(self.data_source_combobox)
        self.source_group.setLayout(source_layout)

        # Board
        self.board_group = QGroupBox("Board options")
        self.board_group.setFlat(True)

        self.board_type_combobox = QComboBox()
        self.board_parameters_button = QPushButton("Board parameters")

        for board_type in BoardEnum:
            self.board_type_combobox.addItem(board_type.value["display"], board_type)

        if self.cfg["board_type"] != "":
            self.board_type_combobox.setCurrentIndex(
                BoardEnum.from_name(self.cfg["board_type"]).get_id()
            )
        else:
            self.board_type_combobox.setCurrentIndex(0)

        def board_parameters_on_closed():
            self.setEnabled(True)

        def board_parameters_button_callback():
            print("board_parameters_button_callback")  # here we whould
            board_parameters_view = ParameterView(self.calibrator.board_parameters)
            # board_parameters_view.parameter_changed.connect(self.on_parameter_changed)
            board_parameters_view.closed.connect(board_parameters_on_closed)
            self.setEnabled(False)

        self.board_parameters_button.clicked.connect(board_parameters_button_callback)

        board_layout = QVBoxLayout()
        board_layout.addWidget(self.board_type_combobox)
        board_layout.addWidget(self.board_parameters_button)
        self.board_group.setLayout(board_layout)

        self.mode_group = QGroupBox("Mode options")
        self.mode_group.setFlat(True)

        self.training_radio_button = QRadioButton("Calibration mode")
        self.evaluation_radio_button = QRadioButton("Evaluation mode")
        self.training_radio_button.setChecked(True)

        mode_layout = QVBoxLayout()
        mode_layout.addWidget(self.training_radio_button)
        mode_layout.addWidget(self.evaluation_radio_button)
        self.mode_group.setLayout(mode_layout)

        self.start_button = QPushButton("Start")
        self.start_button.clicked.connect(self.on_start)

        self.layout.addWidget(self.source_group)
        self.layout.addWidget(self.board_group)
        self.layout.addWidget(self.mode_group)
        self.layout.addWidget(self.start_button)

        self.show()

    def on_start(self):
        def on_success():
            mode = (
                OperationMode.CALIBRATION
                if self.training_radio_button.isChecked()
                else OperationMode.EVALUATION
            )
            board_type = self.board_type_combobox.currentData()
            self.calibrator.start(mode, self.data_source, board_type)
            self.close()

        def on_failed():
            self.setEnabled(True)

        if self.data_source_combobox.currentData() == DataSourceEnum.TOPIC:

            self.data_source = make_data_source(self.data_source_combobox.currentData())
            self.data_source.set_data_callback(self.calibrator.data_source_external_callback)

            self.data_source_view = RosTopicView(self.data_source)
            self.data_source_view.failed.connect(on_failed)
            self.data_source_view.success.connect(on_success)
            self.setEnabled(False)

        else:
            raise NotImplementedError

    def closeEvent(self, event):
        print("Initialization view: closeEvent")
        self.closed.emit()
        event.accept()
        self.deleteLater()

    def __del__(self):
        print("Initialization view: destructor")
