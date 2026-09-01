import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# detect all 16h5 tags
cfg_16h5 = {
    "image_transport": "raw",
    "family": "16h5",
    "size": 0.162,
    "max_hamming": 0,
    "z_up": True,
}


def inferred_camera_info_topic(image_topic: str) -> str:
    """Build the camera_info topic the same way as image_transport::getCameraInfoTopic().

    Replace the last path component of the image topic with camera_info.
    Example: /sensing/camera/camera7/image_raw/decompressed
          -> /sensing/camera/camera7/image_raw/camera_info
    """
    if "/" in image_topic:
        return image_topic.rsplit("/", 1)[0] + "/camera_info"
    return "camera_info"


def launch_setup(context, *args, **kwargs):
    # AprilTagNode uses image_transport::CameraSubscriber, which subscribes to
    # image and camera_info as a pair. The constructor first calls
    # resolve_topic_name("image_rect") and passes that resolved name in, so
    # remapping image_rect works, but camera_info is inferred from the image topic.
    #
    # From image_transport::getCameraInfoTopic() in
    # image_transport/camera_common.hpp:
    #   "This function assumes that the name is completely resolved. If the
    #    base_topic is remapped the resulting camera info topic will be incorrect."
    #
    # Therefore remapping ("camera_info", <actual topic>) has no effect: the
    # subscription is created with the inferred absolute name, not "camera_info".
    #
    # Workaround: remap the inferred name to the camera_info_topic we actually want.
    image_topic = LaunchConfiguration("image_topic").perform(context)
    camera_info_topic = LaunchConfiguration("camera_info_topic").perform(context)
    detections_topic = LaunchConfiguration("apriltag_detections_topic").perform(context)

    composable_node = ComposableNode(
        name="apriltag",
        package="apriltag_ros",
        plugin="AprilTagNode",
        remappings=[
            ("image", image_topic),
            ("image_rect", image_topic),
            (inferred_camera_info_topic(image_topic), camera_info_topic),
            ("detections", detections_topic),
        ],
        parameters=[cfg_16h5, {"qos_profile": "sensor_data"}],
    )

    container = ComposableNodeContainer(
        name="tag_container",
        namespace="apriltag",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[composable_node],
        output="screen",
    )

    return [container]


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("image_topic", "/camera/image")
    add_launch_arg("camera_info_topic", "/camera/camera_info")
    add_launch_arg("apriltag_detections_topic", "apriltag/detection_array")

    return launch.LaunchDescription(
        launch_arguments + [OpaqueFunction(function=launch_setup)]
    )
