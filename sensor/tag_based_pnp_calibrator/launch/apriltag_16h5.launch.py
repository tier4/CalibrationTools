import launch
from launch.actions import DeclareLaunchArgument
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


def generate_launch_description():
    launch_arguments = []

    def add_launch_arg(name: str, default_value=None):
        launch_arguments.append(DeclareLaunchArgument(name, default_value=default_value))

    add_launch_arg("image_topic", "/camera/image")
    add_launch_arg("camera_info_topic", "/camera/camera_info")
    add_launch_arg("apriltag_detections_topic", "apriltag/detection_array")

    composable_node = ComposableNode(
        name="apriltag",
        package="apriltag_ros",
        plugin="AprilTagNode",
        remappings=[
            ("image", LaunchConfiguration("image_topic")),
            ("image_rect", LaunchConfiguration("image_topic")),
            ("camera_info", LaunchConfiguration("camera_info_topic")),
            ("detections", LaunchConfiguration("apriltag_detections_topic")),
        ],
        parameters=[cfg_16h5],
    )

    container = ComposableNodeContainer(
        name="tag_container",
        namespace="apriltag",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[composable_node],
        output="screen",
    )

    return launch.LaunchDescription(launch_arguments + [container])
