import threading

from PySide2.QtCore import Signal
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QFileDialog
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from intrinsic_camera_calibrator.data_sources.ros_bag_data_source import RosBagDataSource


class RosBagView(QWidget):

    failed = Signal()
    success = Signal()

    set_rosbag_request = Signal(str)
    rosbag_start_request = Signal(str)

    def __init__(self, data_source: RosBagDataSource):
        self.data_source = data_source

        super().__init__()

        self.setWindowTitle("Select a ros bag and topic")
        self.setMinimumWidth(300)

        self.bag_selected = False
        self.topic_selected = False

        self.data_source = data_source
        self.set_rosbag_request.connect(self.data_source.set_rosbag_file)
        self.data_source.rosbag_topics_signal.connect(self.update_topics)
        self.rosbag_start_request.connect(self.data_source.start)

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.select_bag_button = QPushButton("Select ROS bag")
        self.layout.addWidget(self.select_bag_button)

        self.label = QLabel("Select topic")
        self.layout.addWidget(self.label)

        self.topic_combo_box = QComboBox()
        self.topic_combo_box.setEnabled(False)
        self.layout.addWidget(self.topic_combo_box)

        self.accept_button = QPushButton("Ok")
        self.accept_button.setEnabled(False)
        self.layout.addWidget(self.accept_button)

        self.select_bag_button.clicked.connect(self.select_bag_callback)
        self.accept_button.clicked.connect(self.accept_callback)

        self.topic_combo_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.show()

    def select_bag_callback(self):

        fname = QFileDialog.getOpenFileName()
        print(f"{threading.get_ident()}: QFiledialog selected = {fname[0]}")
        self.set_rosbag_request.emit(fname[0])

    def update_topics(self, topic_list):

        topic_list.sort()

        for topic in topic_list:
            self.topic_combo_box.addItem(topic)

        if len(topic_list):
            self.topic_combo_box.setEnabled(True)
            self.accept_button.setEnabled(True)

        self.adjustSize()

    def accept_callback(self):

        if self.topic_combo_box.count() == 0:
            return

        image_topic = self.topic_combo_box.currentText()
        self.topic_selected = True

        self.success.emit()
        self.rosbag_start_request.emit(image_topic)
        self.close()

    def closeEvent(self, event):
        print("Ros bag data view: closeEvent")
        if not self.topic_selected:
            self.failed.emit()
        event.accept()

        self.deleteLater()

    def __del__(self):
        print("Ros bag data view: destructor")
