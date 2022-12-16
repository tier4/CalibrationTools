from PySide2.QtCore import Signal
from PySide2.QtWidgets import QComboBox
from PySide2.QtWidgets import QLabel
from PySide2.QtWidgets import QPushButton
from PySide2.QtWidgets import QVBoxLayout
from PySide2.QtWidgets import QWidget
from intrinsic_camera_calibrator.data_sources.ros_topic_data_source import RosTopicDataSource


class RosTopicView(QWidget):

    failed = Signal()
    success = Signal()

    def __init__(self, data_source: RosTopicDataSource):
        self.data_source = data_source

        super().__init__()

        self.setWindowTitle("Select ros topic")
        # self.setAttribute(Qt.WA_DeleteOnClose)

        self.topic_selected = False

        self.layout = QVBoxLayout()
        self.setLayout(self.layout)

        self.label = QLabel("Ros topics:")
        self.layout.addWidget(self.label)

        self.combo_box = QComboBox()
        self.layout.addWidget(self.combo_box)

        self.update_button = QPushButton("Refresh list")
        self.layout.addWidget(self.update_button)

        self.accept_button = QPushButton("Ok")
        self.layout.addWidget(self.accept_button)

        self.update_button.clicked.connect(self.update_list_callback)
        self.accept_button.clicked.connect(self.accept_callback)

        self.update_list_callback()
        self.combo_box.setSizeAdjustPolicy(QComboBox.AdjustToContents)
        self.show()

    def update_list_callback(self):

        image_topics = self.data_source.get_image_topics()

        self.combo_box.clear()

        for image_topic in image_topics:
            self.combo_box.addItem(image_topic)

        self.adjustSize()

    def accept_callback(self):

        if self.combo_box.count() == 0:
            return

        image_topic = self.combo_box.currentText()
        self.topic_selected = True

        self.data_source.set_image_topic(image_topic)
        self.success.emit()
        self.close()

    def closeEvent(self, event):
        print("Ros topic data view: closeEvent")
        if not self.topic_selected:
            self.failed.emit()
        event.accept()

        self.deleteLater()

    def __del__(self):
        print("Ros topic data view: destructor")
