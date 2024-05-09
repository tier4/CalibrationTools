#!/usr/bin/env python3

# Copyright 2024 Tier IV, Inc.
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


from intrinsic_camera_calibrator.data_sources.data_source import DataSource
from intrinsic_camera_calibrator.data_sources.data_source import DataSourceEnum
from intrinsic_camera_calibrator.data_sources.image_files_data_source import ImageFilesDataSource
from intrinsic_camera_calibrator.data_sources.ros_bag_data_source import RosBagDataSource
from intrinsic_camera_calibrator.data_sources.ros_topic_data_source import RosTopicDataSource

# from intrinsic_camera_calibrator.data_sources.video_file_data_source import VideoFileDataSource


def make_data_source(source_type: DataSourceEnum, **kwargs) -> DataSource:
    """Create a DataSource using a factory design pattern."""
    classes_dic = {
        DataSourceEnum.TOPIC: RosTopicDataSource,
        DataSourceEnum.BAG2: RosBagDataSource,
        # DataSourceEnum.VIDEO: VideoFileDataSource,
        DataSourceEnum.FILES: ImageFilesDataSource,
    }
    return classes_dic[source_type](**kwargs)
