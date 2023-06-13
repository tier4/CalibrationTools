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

import signal
import sys

from PySide2.QtWidgets import QApplication
from extrinsic_reflector_based_calibrator import CalibratorUI
from extrinsic_reflector_based_calibrator import RosInterface
import rclpy


def main(args=None):
    app = QApplication(sys.argv)

    rclpy.init(args=args)

    try:
        signal.signal(signal.SIGINT, sigint_handler)

        ros_interface = RosInterface()
        window = CalibratorUI(ros_interface)  # noqa: F841

        ros_interface.spin()
        sys.exit(app.exec_())

    except (KeyboardInterrupt, SystemExit):
        print("Received sigint. Quiting...", flush=True)
        rclpy.shutdown()


def sigint_handler(*args):
    QApplication.quit()


if __name__ == "__main__":
    main()
