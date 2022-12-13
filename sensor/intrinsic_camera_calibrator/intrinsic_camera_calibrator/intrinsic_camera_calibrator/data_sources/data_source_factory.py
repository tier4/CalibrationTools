from intrinsic_camera_calibrator.data_sources.data_source import DataSource
from intrinsic_camera_calibrator.data_sources.data_source import DataSourceEnum
from intrinsic_camera_calibrator.data_sources.image_files_data_source import ImageFilesDataSource
from intrinsic_camera_calibrator.data_sources.ros_bag_data_source import RosBagDataSource
from intrinsic_camera_calibrator.data_sources.ros_topic_data_source import RosTopicDataSource
from intrinsic_camera_calibrator.data_sources.video_file_data_source import VideoFileDataSource


def make_data_source(source_type: DataSourceEnum, **kwargs) -> DataSource:
    classes_dic = {
        DataSourceEnum.TOPIC: RosTopicDataSource,
        DataSourceEnum.BAG2: RosBagDataSource,
        DataSourceEnum.VIDEO: VideoFileDataSource,
        DataSourceEnum.FILES: ImageFilesDataSource,
    }
    return classes_dic[source_type](**kwargs)
