# Copyright 2024 Tier IV, Inc. All rights reserved.
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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import yaml


def launch_setup(context, *args, **kwargs):
    def create_parameter_dict(*args):
        result = {}
        for x in args:
            result[x] = LaunchConfiguration(x)
        return result

    common_param_dict = create_parameter_dict(
        "image_transport",
        "max_hamming",
        "z_up",
    )

    families = yaml.safe_load(LaunchConfiguration("families").perform(context))

    nodes = []

    for family in families:
        param_dict = {"family": family, **common_param_dict}

        composable_node = ComposableNode(
            name=f"apriltag_{family}",
            package="apriltag_ros",
            plugin="AprilTagNode",
            remappings=[
                ("image", LaunchConfiguration("image_topic")),
                ("image_rect", LaunchConfiguration("image_topic")),
                ("camera_info", LaunchConfiguration("camera_info_topic")),
                ("detections", LaunchConfiguration("apriltag_detections_topic")),
            ],
            parameters=[
                {
                    **param_dict,
                }
            ],
        )

        nodes.append(composable_node)

    container = ComposableNodeContainer(
        name="tag_container",
        namespace="apriltag",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=nodes,
        output="screen",
    )

    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None, description=None):
        # a default_value of None is equivalent to not passing that kwarg at all
        launch_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value, description=description)
        )

    add_launch_arg("image_topic", "/camera/image")
    add_launch_arg("camera_info_topic", "/camera/camera_info")
    add_launch_arg("apriltag_detections_topic", "apriltag/detection_array")
    add_launch_arg("image_transport", "raw")
    add_launch_arg("families", "[16h5]")
    add_launch_arg("max_hamming", "0")
    add_launch_arg("z_up", "True")

    set_container_executable = SetLaunchConfiguration(
        "container_executable",
        "component_container",
    )

    return launch.LaunchDescription(
        launch_arguments + [set_container_executable] + [OpaqueFunction(function=launch_setup)]
    )
