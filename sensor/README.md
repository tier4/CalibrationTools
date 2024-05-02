# Sensor Calibration Tools

Calibration tools for sensors used in autonomous driving and robotics (camera, lidar, and radar).

## Table of contents

- [Installation](#installation)
  - [Requirements](#requirements)
  - [Installation alongside autoware](#installation-alongside-autoware)
  - [Standalone installation (for non-autoware users)](#standalone-installation-for-non-autoware-users)
  - [Standalone installation using Docker (for non-autoware users)](#standalone-installation-using-docker-for-non-autoware-users)
- [Implemented tools](#implemented-tools)
  - [Extrinsic calibration tools](#extrinsic-calibration-tools)
  - [Intrinsic calibration tools](#intrinsic-calibration-tools)
- [Design](#design)
  - [Calibrator node](#calibrator-node)
  - [Sensor calibrator manager](#sensor-calibration-manager)
    - [Projects and calibrators](#projects-and-calibrators)
    - [Calibrator interface](#calibrator-interface)
    - [Launch files](#launch-files)
    - [Launching the sensor calibration manager](#launching-the-sensor-calibration-manager)
- [Integration](#integration)
  - [Using your vehicle/robot](#using-your-vehiclerobot)
  - [Create a new project](#create-a-new-project)
  - [Integrate a new calibrator](#integrate-a-new-calibrator)

## Installation

### Requirements

- Ubuntu 22.04
- ROS2 Humble

### Installation alongside autoware

After installing [autoware](https://github.com/tier4/autoware) (please see [source-installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) page), execute the following commands:

```bash
cd autoware
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools.repos
vcs import src < calibration_tools.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Standalone installation (for non-autoware users)

The sensor calibration tools are usually used as part of the autoware ecosystem. However, they can also be used for projects outside autoware, or even outside autonomous driving. Note: due to its use in autoware, even if it is possible to use the sensor calibration tools independently, due to some light dependencies, parts of autoware still need to be downloaded, even if they are not all compiled.

The following commands present an example of how to install the calibration tools and their dependencies assuming you have a ROS2 workspace called `workspace`:

```bash
# Install vcs (if needed, follow the instructions from https://github.com/dirk-thomas/vcstool)
sudo apt-get install python3-vcstool

# Download the calibration tools and its dependencies
cd workspace
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools_standalone.repos
vcs import src < calibration_tools_standalone.repos

# Install all the dependencies from rosdep
rosdep install -y --from-paths `colcon list --packages-up-to sensor_calibration_tools -p` --ignore-src

# Build the sensor calibration tools
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-up-to sensor_calibration_tools
```

### Standalone installation using Docker (for non-autoware users)

With a similar motivation to that of the previous Section, in some cases, a native build is not possible nor convenient. To accommodate to those situations, we also offer the sensor calibration tools as a docker image:

```bash
# Build
DOCKER_BUILDKIT=1 docker build --ssh default -t ghcr.io/tier4/calibration-tools:2.0 -f docker/Dockerfile ..

# Run - Modify if needed
docker run --gpus all --net=host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri:/dev/dri -it ghcr.io/tier4/calibration-tools:2.0 /bin/bash
```

## Implemented tools

### Extrinsic calibration tools

| Name                                | Sensors calibrated | Feature type                | Calibration type   | Documentation                                   | Tutorial                                                |
| ----------------------------------- | ------------------ | --------------------------- | ------------------ | ----------------------------------------------- | ------------------------------------------------------- |
| ground plane calibrator             | base-lidar         | ground                      | roll, pitch, z     | [Link](ground_plane_calibrator/README.md)       | N/A                                                     |
| interactive camera-lidar calibrator | camera-lidar       | manual correspondences      | full pose          | Link                                            | N/A                                                     |
| lidar-lidar 2d calibrator           | lidar-lidar        | natural features            | x, y, yaw          | Link                                            | N/A                                                     |
| mapping calibrator (lidar-lidar)    | lidar-lidar        | natural features            | full pose          | [Link](mapping_based_calibrator/README.md)      | [Link](docs/tutorials/mapping_based_calibrator.md)      |
| mapping calibrator (base-lidar)     | base-lidar         | natural features and ground | roll, pitch, and z | Link                                            | N/A                                                     |
| marker radar-lidar calibrator       | radar-lidar        | marker                      | x, y, yaw          | [Link](marker_radar_lidar_calibrator/README.md) | [Link](docs/tutorials/marker_radar_lidar_calibrator.md) |
| tag-based PnP calibrator            | camera-lidar       | marker                      | full pose          | [Link](tag_based_pnp_calibrator/README.md)      | [Link](docs/tutorials/tag_based_pnp_calibrator.md)      |
| tag-based SfM calibrator            | camera-lidar-base  | marker                      | full pose          | [Link](tag_based_sfm_calibrator/README.md)      | [Link](docs/tutorials/tag_based_sfm_calibrator.md)      |

### Intrinsic calibration tools

| Name                         | Sensors calibrated | Feature type       | Calibration type    | Demo |
| ---------------------------- | ------------------ | ------------------ | ------------------- | ---- |
| camera intrinsics calibrator | camera intrinsics  | calibration boards | OpenCV camera model | N/A  |

## Design

The sensor calibration tools repository provides different alternatives for both intrinsic and extrinsic calibration. That being said, this document focuses only on extrinsic calibration since camera intrinsic calibration is a fairly simple and easy-to-understand process.

The architecture of the extrinsic calibration process consists of two entities: the calibrator node itself and the sensor calibration manager (additional nodes may be used, but they do not participate directly in the calibration process). In what follows, we proceed to detail the roles of each one of these elements.

### Calibrator node

The calibrator node is a regular node that implements the [ExtrinsicCalibrator](../common/tier4_calibration_msgs/srv/ExtrinsicCalibrator.srv) service:

```text
---
tier4_calibration_msgs/CalibrationResult[] results
```

where [CalibrationResult](../common/tier4_calibration_msgs/msg/CalibrationResult.msg) contains a transformation between frames, a status flag, and optional scores and text messages for evaluation and debug purposes.

```text
geometry_msgs/TransformStamped transform_stamped
bool success
float32 score
std_msgs/String message
```

The design is intended to decouple the calibrator node and its internal logic as much as possible from the details of a particular calibration use-case and the `tf` structure used.
The code of the calibrator itself is agnostic to everything other than its particular task, with the service request not even containing the frames to calibrate. This way, all the configuration needs to happen in the external node configuration and the sensor calibration manager (optional) to achieve a high level of separation of concerns.

### Sensor calibration manager

Although the calibrator process can be performed directly via launching the calibrator node and using the service interface using the CLI, it is highly recommended to automate the process using the sensor calibration manager.
The sensor calibrator manager consists of a UI that allows the user to select a particular combination of project and calibrator, make sure that the required `tf` and services are available, and process/save the calibration results.

#### Projects and calibrators

At Tier IV we currently run several projects that use several types of sensors. Due to this, we do not create calibrators for each particular project, and instead reuse the same code, only modifying the parameters and helper nodes. For this reason, in the sensor calibration manager, we introduce the concepts of projects and calibrators. In this context, a project consists of a list of calibrators, with the same calibrator (semantically) being able to belong to multiple projects.

An example of this structure is as follows:

```text
calibrators/
calibrators/projectA/
calibrators/projectA/calibratorA.py
calibrators/projectA/calibratorB.py
calibrators/projectB/
calibrators/projectB/calibratorA.py
calibrators/projectB/calibratorB.py
calibrators/projectC/
calibrators/projectC/calibratorC.py
```

#### Calibrator interface

A calibrator interface is the representation of the calibration process inside the sensor calibration manager. It specifies its project, the calibrator name, the `tf` that are required during the calibration process, and the expected frames that the calibrator node should return.

Following the previous example, the `calibratorA.py` could be implemented as follows:

```python
@CalibratorRegistry.register_calibrator(
    project_name="projectA", calibrator_name="calibratorA"
)
class CalibratorA(CalibratorBase):
    required_frames = ["calibration_parent_frame", "calibration_child_frame", "auxiliar_frame"]

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.add_calibrator(
            service_name="the_name_of_the_calibration_service",
            expected_calibration_frames=[
                FramePair(parent="calibration_parent_frame", child="calibration_child_frame"),
            ],
        )

```

In addition to specifying `required_frames` and `services_name`, the calibrator interfaces are also used to post-process the calibration results if needed to conform to robotics frame conventions and other project-specific requirements.
For example, camera-lidar calibration returns the `tf` from the optical link to the lidar itself. However, in most scenarios instead of the optical link, the camera link is preferred in configuration files (the camera link has different axes), and some lidars prefer to use their `base_link` (footprint).

For example, at Tier IV, most sensors are mounted in a structure called `sensor_kit`, and then the sensor calibration is represented as a `sensor_kit_base_link -> sensor_frame`. In particular, for the case of camera-lidar, the `tf` that represents the camera-lidar calibration in most of our projects is `sensor_kit_base_link -> cameraX/camera_link`. To transform the `tf` that the calibrator returns (`lidar -> cameraX/camera_optical_link`) to the one we need to save, the post-process step can be implemented as follows:

```python
# Taken from sensor_calibration_manager/sensor_calibration_manager/calibrators/xx1/tag_based_pnp_calibrator.py
def post_process(self, calibration_transforms: Dict[str, Dict[str, np.array]]):
    optical_link_to_lidar_transform = calibration_transforms[
        f"{self.camera_name}/camera_optical_link"
    ]["velodyne_top"]
    sensor_kit_to_lidar_transform = self.get_transform_matrix(
        "sensor_kit_base_link", "velodyne_top"
    )
    camera_to_optical_link_transform = self.get_transform_matrix(
        f"{self.camera_name}/camera_link", f"{self.camera_name}/camera_optical_link"
    )
    sensor_kit_camera_link_transform = np.linalg.inv(
        camera_to_optical_link_transform
        @ optical_link_to_lidar_transform
        @ np.linalg.inv(sensor_kit_to_lidar_transform)
    )

    result = {
        "sensor_kit_base_link": {
            f"{self.camera_name}/camera_link": sensor_kit_camera_link_transform
        }
    }
    return result
```

#### Launch files

The calibrator interface does not implement any of the ROS logic in terms of the involved nodes. This part of the process is implemented by regular launch files that are called by the sensor calibration manager.

Following the previous example, the launcher structure would be as follows:

```text
launch/
launch/projectA/
launch/projectA/calibratorA.launch.xml
launch/projectA/calibratorB.launch.xml
launch/projectB/
launch/projectB/calibratorA.launch.xml
launch/projectB/calibratorB.launch.xml
launch/projectC/
launch/projectC/calibratorC.launch.xml
```

The launch file can have arguments with and without default arguments that will be automatically transformed into a configurable UI so the user can set them during start-up.
One point of note is that the service specified in the calibrator interface must be offered by a node in the launch file.

#### Launching the sensor calibration manager

To execute the calibration manager, the user just the to run the following (after sourcing the ROS workspace):

```bash
ros2 run sensor_calibration_manager sensor_calibration_manager
```

Upon executing the last command, the following UI should appear:

<p align="center">
  <img src="docs/images/sensor_calibration_manager/initial_menu.png" alt="initial_menu"/>
</p>

Then, the user must select a combination of project and calibrator and press `Continue`.
After this, the following UI should be displayed:

<p align="center">
  <img src="docs/images/sensor_calibration_manager/launcher_configuration.png" alt="launcher_configuration"/>
</p>

Here, the user must configure the launcher arguments as required. Since we are using the combination of `default_project` and `tag_based_pnp_calibrator`, the launcher file being parameterized under the hood is `sensor_calibration_manager/launch/default_project/tag_based_pnp_calibrator.launch.xml`. Once the user finishes setting the parameter he must click the `Launch` button.

After this, the sensor calibrator manager will execute the previous launch file with the corresponding parameters and internally use `sensor_calibration_manager/sensor_calibration_manager/calibrators/default_project/tag_based_pnp_calibrator.py` as its calibrator interface. The following window will be displayed:

<p align="center">
  <img src="docs/images/sensor_calibration_manager/main_window.png" alt="main_window"/>
</p>

If the calibrator node launches successfully, its service becomes available, and the required `tf` are all present, the `calibrate` button should become enabled. Clicking this calls the calibration service, and starts the process.

For visualization purposes, the sensor calibration manager displays the required `tf` specified in the calibrator interface as shown in the following image:

<p align="center">
  <img src="docs/images/sensor_calibration_manager/initial_tf.png" alt="initial_tf"/>
</p>

Once the calibration finishes, the result from the `ExtrinsicCalibrator` is displayed in the `Calibration tree` widget as shown in the following image:

<p align="center">
  <img src="docs/images/sensor_calibration_manager/calibrated_tf.png" alt="calibrated_tf"/>
</p>

If the calibrator interface had post-processing steps, the `Final TF tree` widget would show the processed results. In this case, since there is not a post-process step, both widgets are the same.

Finally, to save the results, press the `save calibration` button.

<p align="center">
  <img src="docs/images/sensor_calibration_manager/result.png" alt="result"/>
</p>

## Integration

### Using your vehicle/robot

Although we provide several projects and examples, in most cases the user would need to modify several parts of this repository to create their calibration projects. To ease this process, we also created a `default_project` that exposes most of each calibrator's parameters so that users can use this repository without creating any new files. In turn, however, they must properly configure most of the calibrator's options in the `Launcher configuration` widget, which is mostly done automatically in our internal projects.

### Create a new project

If the default project is not enough or does not meet the needs of the user (for example, if they are going to start projects that require running the tools frequently), it is relatively easy to create new projects and calibrator interfaces. In what follows, we will create a new project titled `my_new_project` with a calibrator called `my_new_calibrator`.

First, to make sure that the sensor calibration manager knows of the existence of the calibrator, add the following line to `sensor_calibration_manager/sensor_calibration_manager/calibrators/__init__.py`:

```python
from .my_new_project import *  # noqa: F401, F403
```

Then, create a file called `sensor_calibration_manager/sensor_calibration_manager/calibrators/my_new_project/__init__.py`.

```python
from .my_new_calibrator import MyNewCalibrator

__all__ = [
    "MyNewCalibrator",
]
```

After this, create a file called `sensor_calibration_manager/sensor_calibration_manager/calibrators/xx1_15/tag_based_pnp_calibrator.py` with the following contents:

```python
from sensor_calibration_manager.calibrator_base import CalibratorBase
from sensor_calibration_manager.calibrator_registry import CalibratorRegistry
from sensor_calibration_manager.ros_interface import RosInterface
from sensor_calibration_manager.types import FramePair


@CalibratorRegistry.register_calibrator(
    project_name="my_new_project", calibrator_name="my_new_calibrator"
)
class MyNewCalibrator(CalibratorBase):
    required_frames = []

    def __init__(self, ros_interface: RosInterface, **kwargs):
        super().__init__(ros_interface)

        self.source_frame: str = kwargs["source_frame"]
        self.target_frame: str = kwargs["target_frame"]

        self.required_frames.extend([self.base_frame, self.source_frame, self.target_frame])

        self.add_calibrator(
            service_name="calibrate_service_name",
            expected_calibration_frames=[
                FramePair(parent=self.target_frame, child=self.source_frame),
            ],
        )

```

This calibrator will expect a calibrator between `source_frame` and `target_frame`, which are parameters provided by the user through the launcher file.

Finally, the user must create the correspondent launch file `sensor_calibration_manager/launch/my_new_project/my_new_calibrator.launch.xml`.
The launcher itself must contain the following arguments to match the calibrator interface:

```xml
<arg name="source_frame"/>
<arg name="target_frame"/>
```

Default values are optional, and somewhere inside the launch file, the user needs to execute a node that provides the `calibrate_service_name` to match the calibrator interface.

### Integrate a new calibrator

In the previous Section, we created a new calibrator interface and added it to the sensor calibration manager. However, that assumes that the user will use one of the calibrator packages offered by the sensor calibration tools.

In the case the user wants to integrate his algorithms, he must follow the instructions, which assume the reader knows and is used to creating ROS2 packages.

- Create a ROS2 package called `my_new_calibrator_package`. The node itself needs to be part of a muli-thread executor with at least two threads.
- Add a dependency to the `tier4_calibration_msgs` package to use the calibration services.
- In the node's header file add a calibration service.
- In most cases, creating a group exclusive to the services is also required.

In the header file:

```c++
...
rclcpp::Service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>::SharedPtr service_server_;
rclcpp::CallbackGroup::SharedPtr srv_callback_group_
...
```

In the source file:

```c++
...
// The service server runs in a dedicated thread since it is a blocking call
srv_callback_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

service_server_ = this->create_service<tier4_calibration_msgs::srv::ExtrinsicCalibrator>(
  "extrinsic_calibration_service_name",
  std::bind(
    &MyNewCalibratorPackage::requestReceivedCallback, this, std::placeholders::_1,
    std::placeholders::_2),
  rmw_qos_profile_services_default, srv_callback_group_);
  ...
```

In addition to this, the user must implement `requestReceivedCallback` to comply with the interface.
