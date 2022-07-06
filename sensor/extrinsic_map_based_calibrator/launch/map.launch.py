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

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    map_hash_generator = Node(
        package="map_loader",
        executable="map_hash_generator",
        name="map_hash_generator",
        parameters=[
            {
                "lanelet2_map_path": LaunchConfiguration("lanelet2_map_path"),
                "pointcloud_map_path": LaunchConfiguration("pointcloud_map_path"),
            }
        ],
    )

    pointcloud_map_loader = ComposableNode(
        package="map_loader",
        plugin="PointCloudMapLoaderNode",
        name="pointcloud_map_loader",
        remappings=[("output/pointcloud_map", "/map/pointcloud_map_without_wall")],
        parameters=[
            {"pcd_paths_or_directory": ["[", LaunchConfiguration("pointcloud_map_path"), "]"]},
            {"map_frame": "map"}
        ],
        extra_arguments=[{"use_intra_process_comms": LaunchConfiguration("use_intra_process")}],
    )

    container = ComposableNodeContainer(
        name="map_container",
        namespace="",
        package="rclcpp_components",
        executable=LaunchConfiguration("container_executable"),
        composable_node_descriptions=[
            pointcloud_map_loader,
        ],
        output="screen",
    )

    def add_launch_arg(name: str, default_value=None, description=None):
        return DeclareLaunchArgument(name, default_value=default_value, description=description)

    return launch.LaunchDescription(
        [
            add_launch_arg("map_path", "", "path to map directory"),
            add_launch_arg(
                "lanelet2_map_path",
                [LaunchConfiguration("map_path"), "/lanelet2_map.osm"],
                "path to lanelet2 map file",
            ),
            add_launch_arg(
                "pointcloud_map_path",
                [LaunchConfiguration("map_path"), "/pointcloud_map.pcd"],
                "path to pointcloud map file",
            ),
            add_launch_arg(
                "use_intra_process", "false", "use ROS2 component container communication"
            ),
            add_launch_arg("use_multithread", "false", "use multithread"),
            SetLaunchConfiguration(
                "container_executable",
                "component_container",
                condition=UnlessCondition(LaunchConfiguration("use_multithread")),
            ),
            SetLaunchConfiguration(
                "container_executable",
                "component_container_mt",
                condition=IfCondition(LaunchConfiguration("use_multithread")),
            ),
            GroupAction(
                [
                    PushRosNamespace("map"),
                    container,
                    map_hash_generator,
                ]
            ),
        ]
    )
