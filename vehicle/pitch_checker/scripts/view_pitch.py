#! /usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright 2021 Tier IV, Inc. All rights reserved.
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

from ament_index_python.packages import get_package_share_directory
import matplotlib.pyplot as plt
import pandas as pd
import rclpy
from rclpy.node import Node


class ViewPlot(Node):

    def __init__(self):
        super().__init__('pitch_viewer')
        package_path = get_package_share_directory('pitch_checker')
        self.declare_parameter('pitch_file_name', package_path + '/pitch.csv')
        file_name = self.get_parameter('pitch_file_name').get_parameter_value().string_value
        data = pd.read_csv(file_name)

        x = data['x']
        y = data['y']
        z = data['z']
        pitch = data['pitch']

        fig1 = plt.subplot(121)
        # cmap... cmap or hsv
        sc1 = fig1.scatter(x, y, c=pitch, label='pitch', cmap='gist_rainbow')
        sc1.set_clim(-0.025, 0.025)
        plt.colorbar(sc1)
        fig1.set_title('pitch')

        fig2 = plt.subplot(122)
        sc2 = fig2.scatter(x, y, c=z, label='z', cmap='gist_rainbow')
        plt.colorbar(sc2)
        fig2.set_title('z-axis')
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = ViewPlot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
