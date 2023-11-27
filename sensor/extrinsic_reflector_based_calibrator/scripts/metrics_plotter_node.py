#!/usr/bin/env python3

# Copyright 2023 Tier IV, Inc.
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

#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import matplotlib.pyplot as plt

class Plotter:
    def __init__(self):
        self.fig, self.axes = plt.subplots(nrows=2, ncols=2, figsize=(15, 12))

        self.prev_calibration_distance_error = 0
        self.prev_calibration_yaw_error = 0
        self.prev_cv_distance_error = 0
        self.prev_cv_yaw_error = 0
        self.prev_num_of_reflectors = 0
        self.anno0, self.anno1, self.anno2, self.anno3 = None, None, None, None
        self.max_ylim0, self.max_ylim1, self.max_ylim2, self.max_ylim3 = 0, 0, 0, 0
        #self.anno_list0, self.anno_list1, self.anno_list2, self.anno_list3 = [],[],[],[]
        self.plot_setting()
        plt.pause(0.1)
        

    def plot_setting(self):
        self.axes[0, 0].set_title('Cross val error: Distance')
        self.axes[0, 0].set_xlabel('# reflector')
        self.axes[0, 0].set_ylabel('Distance error')

        self.axes[0, 1].set_title('Cross val error: Yaw')
        self.axes[0, 1].set_xlabel('# reflector')
        self.axes[0, 1].set_ylabel('Yaw error')

        self.axes[1, 0].set_title('Average error: Distance')
        self.axes[1, 0].set_xlabel('# reflector')
        self.axes[1, 0].set_ylabel('Distance error')

        self.axes[1, 1].set_title('Average error: Distance')
        self.axes[1, 1].set_xlabel('# reflector')
        self.axes[1, 1].set_ylabel('Yaw error')


    def remove_anno(self, anno):
        if(anno is not None):
            anno.remove()


    def draw_graph(self, msg):
        num_of_reflectors = msg.data[0]
        cv_distance_error = msg.data[1]
        cv_yaw_error = msg.data[2]
        calibration_distance_error = msg.data[3]
        calibration_yaw_error = msg.data[4]

        # update the maxium ylim
        self.max_ylim0 = cv_distance_error if cv_distance_error > self.max_ylim0 else self.max_ylim0
        self.max_ylim1 = cv_yaw_error if cv_distance_error > self.max_ylim1 else self.max_ylim1
        self.max_ylim2 = calibration_distance_error if cv_distance_error > self.max_ylim2 else self.max_ylim2
        self.max_ylim3 = calibration_yaw_error if cv_distance_error > self.max_ylim3 else self.max_ylim3

        # remove the previous annotations
        self.remove_anno(self.anno0)
        self.remove_anno(self.anno1)
        self.remove_anno(self.anno2)
        self.remove_anno(self.anno3)

        # make the plot dynamic
        self.axes[0, 0].set_xlim(0, num_of_reflectors)
        self.axes[0, 1].set_xlim(0, num_of_reflectors)
        self.axes[1, 0].set_xlim(0, num_of_reflectors)
        self.axes[1, 1].set_xlim(0, num_of_reflectors)

        self.axes[0, 0].set_ylim(0, self.max_ylim0 + 0.1)
        self.axes[0, 1].set_ylim(0, self.max_ylim1 + 0.1)
        self.axes[1, 0].set_ylim(0, self.max_ylim2 + 0.1)
        self.axes[1, 1].set_ylim(0, self.max_ylim3 + 0.1)

        self.axes[0, 0].plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_cv_distance_error, cv_distance_error], 'bo-')
        self.axes[0, 1].plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_cv_yaw_error, cv_yaw_error], 'go-')
        self.axes[1, 0].plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_calibration_distance_error, calibration_distance_error], 'bo-')
        self.axes[1, 1].plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_calibration_yaw_error, calibration_yaw_error], 'go-')

        self.anno0 = self.axes[0, 0].annotate('%0.4f' % cv_distance_error, xy=(num_of_reflectors, cv_distance_error), color='b')
        self.anno1 = self.axes[0, 1].annotate('%0.4f' % cv_yaw_error, xy=(num_of_reflectors, cv_yaw_error), color='g')
        self.anno2 = self.axes[1, 0].annotate('%0.4f' % calibration_distance_error, xy=(num_of_reflectors, calibration_distance_error), color='b')
        self.anno3 = self.axes[1, 1].annotate('%0.4f' % calibration_yaw_error, xy=(num_of_reflectors, calibration_yaw_error), color='g')
        plt.pause(0.1)

        self.prev_cv_distance_error = cv_distance_error
        self.prev_cv_yaw_error = cv_yaw_error
        self.prev_calibration_distance_error = calibration_distance_error
        self.prev_calibration_yaw_error = calibration_yaw_error
        self.prev_num_of_reflectors = num_of_reflectors


class MetricsPlotter(Node):
    def __init__(self):
        super().__init__('plot_metric')
        self.plotter = Plotter()
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'cross_validation_metrics',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.plotter.draw_graph(msg)


def main(args=None):
    rclpy.init(args=args)
    metrics_plotter = MetricsPlotter()
    rclpy.spin(metrics_plotter)
    metrics_plotter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

