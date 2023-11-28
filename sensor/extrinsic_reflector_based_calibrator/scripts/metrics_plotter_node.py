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
        self.subplot0 = self.axes[0, 0]
        self.subplot1 = self.axes[0, 1]
        self.subplot2 = self.axes[1, 0]
        self.subplot3 = self.axes[1, 1]

        self.cv_distance_error_list = [0]
        self.cv_yaw_error_list = [0]
        self.calibration_distance_error_list = [0]
        self.calibration_yaw_error_list = [0]
        self.num_of_reflectors_list = [0]

        self.color_bo = 'bo-'
        self.color_go = 'go-'
        self.color_b = 'b'
        self.color_g = 'g'
        
        self.prev_calibration_distance_error = 0
        self.prev_calibration_yaw_error = 0
        self.prev_cv_distance_error = 0
        self.prev_cv_yaw_error = 0
        self.prev_num_of_reflectors = 0
        self.anno0, self.anno1, self.anno2, self.anno3 = None, None, None, None
        self.max_ylim0, self.max_ylim1, self.max_ylim2, self.max_ylim3 = 0, 0, 0, 0
        self.line0, self.line1, self.line2, self.line3 = None, None, None, None
        
        self.plot_setting()
        plt.pause(0.1)
        

    def plot_setting(self):
        self.subplot0.set_title('Cross val error: Distance')
        self.subplot0.set_xlabel('# reflector')
        self.subplot0.set_ylabel('Distance error')

        self.subplot1.set_title('Cross val error: Yaw')
        self.subplot1.set_xlabel('# reflector')
        self.subplot1.set_ylabel('Yaw error')

        self.subplot2.set_title('Average error: Distance')
        self.subplot2.set_xlabel('# reflector')
        self.subplot2.set_ylabel('Distance error')

        self.subplot3.set_title('Average error: Distance')
        self.subplot3.set_xlabel('# reflector')
        self.subplot3.set_ylabel('Yaw error')


    def remove_anno(self, anno):
        if(anno is not None):
            anno.remove()

    def remove_line(self, line):
        if(line is not None):
            line.remove()

    
    def redraw_subplot(self, num_of_reflectors, error_list, subplot, color_o):
        subplot.clear()
        self.plot_setting()
        for index, num in enumerate(self.num_of_reflectors_list):
            if index == 0:
                continue
            if num < num_of_reflectors:
                # redraw the subplot
                subplot.plot([self.num_of_reflectors_list[index-1], self.num_of_reflectors_list[index]], [error_list[index-1], error_list[index]], color_o)
                prev_error = error_list[index]
                prev_num = num
            else:
                del error_list[index:len(error_list)-1]
                del self.num_of_reflectors_list[index:len(self.num_of_reflectors_list)-1]
                break

        return prev_error, prev_num


    def draw_graph(self, msg):
        num_of_reflectors = msg.data[0]
        cv_distance_error = msg.data[1]
        cv_yaw_error = msg.data[2]
        calibration_distance_error = msg.data[3]
        calibration_yaw_error = msg.data[4]

        # check  if it is delete operation
        if num_of_reflectors < self.prev_num_of_reflectors: 
            self.prev_cv_distance_error, self.prev_num_of_reflectors = self.redraw_subplot(num_of_reflectors, self.cv_distance_error_list ,self.subplot0, self.color_bo)
            self.prev_cv_yaw_error, _ = self.redraw_subplot(num_of_reflectors, self.cv_yaw_error_list ,self.subplot1, self.color_go)
            self.prev_calibration_distance_error, _ = self.redraw_subplot(num_of_reflectors, self.calibration_distance_error_list ,self.subplot2, self.color_bo)
            self.prev_calibration_yaw_error, _ = self.redraw_subplot(num_of_reflectors, self.calibration_yaw_error_list ,self.subplot3, self.color_go)


        self.cv_distance_error_list.append(cv_distance_error)
        self.cv_yaw_error_list.append(cv_yaw_error)
        self.calibration_distance_error_list.append(calibration_distance_error)
        self.calibration_yaw_error_list.append(calibration_yaw_error)
        self.num_of_reflectors_list.append(num_of_reflectors)
        
        # update the maxium ylim
        self.max_ylim0 = cv_distance_error if cv_distance_error > self.max_ylim0 else self.max_ylim0
        self.max_ylim1 = cv_yaw_error if cv_distance_error > self.max_ylim1 else self.max_ylim1
        self.max_ylim2 = calibration_distance_error if cv_distance_error > self.max_ylim2 else self.max_ylim2
        self.max_ylim3 = calibration_yaw_error if cv_distance_error > self.max_ylim3 else self.max_ylim3

        # make the plot dynamic
        self.subplot0.set_xlim(0, num_of_reflectors)
        self.subplot1.set_xlim(0, num_of_reflectors)
        self.subplot2.set_xlim(0, num_of_reflectors)
        self.subplot3.set_xlim(0, num_of_reflectors)

        self.subplot0.set_ylim(0, self.max_ylim0 + 0.1)
        self.subplot1.set_ylim(0, self.max_ylim1 + 0.1)
        self.subplot2.set_ylim(0, self.max_ylim2 + 0.1)
        self.subplot3.set_ylim(0, self.max_ylim3 + 0.1)

        # remove the previous annotations
        self.remove_anno(self.anno0)
        self.remove_anno(self.anno1)
        self.remove_anno(self.anno2)
        self.remove_anno(self.anno3)

        # draw the lines
        self.line0 = self.subplot0.plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_cv_distance_error, cv_distance_error], self.color_bo)
        self.line1 = self.subplot1.plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_cv_yaw_error, cv_yaw_error], self.color_go)
        self.line2 = self.subplot2.plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_calibration_distance_error, calibration_distance_error], self.color_bo)
        self.line3 = self.subplot3.plot([self.prev_num_of_reflectors, num_of_reflectors], [self.prev_calibration_yaw_error, calibration_yaw_error], self.color_go)

        # write the error value on screen
        self.anno0 = self.subplot0.annotate('%0.4f' % cv_distance_error, xy=(num_of_reflectors, cv_distance_error), color=self.color_b)
        self.anno1 = self.subplot1.annotate('%0.4f' % cv_yaw_error, xy=(num_of_reflectors, cv_yaw_error), color=self.color_g)
        self.anno2 = self.subplot2.annotate('%0.4f' % calibration_distance_error, xy=(num_of_reflectors, calibration_distance_error), color=self.color_b)
        self.anno3 = self.subplot3.annotate('%0.4f' % calibration_yaw_error, xy=(num_of_reflectors, calibration_yaw_error), color=self.color_g)
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

