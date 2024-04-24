# Sensor Calibration Tools

Calibration tools for sensors used in autonomous driving and robotics (camera, lidar, and radar)

## Summary

- [Installation](#installation)
  - [Requirements](#requirements)
  - [Installation alongside autoware](#installation-alongside-autoware)
  - [Standalone installation (for non-autoware users)](#standalone-installation-for-non-autoware-users)
  - [Standalone installation using Docker (for non-autoware users)](#standalone-installation-using-docker-for-non-autoware-users)
- [Implemented tools](#implemented-tools)
  - [Extrinsic calibration tools](#extrinsic-calibration-tools)
  - [Intrinsic calibration tools](#intrinsic-calibration-tools)
- [Design](#design)
- [Integration](#integration)
  - Integrate the calibration tools to your own projects
  - Integrate your own calibration tool

## Installation

### Requirements

- Ubuntu 22.04
- ROS2 Humble

### Installation alongside Autoware

After installing [autoware](https://github.com/tier4/autoware) (please see [source-installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) page), execute the following commands:

```bash
cd autoware
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools.repos
vcs import src < calibration_tools.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Standalone installation (for non-autoware users)

The sensor calibration tools are usually used as part of the Autoware ecosystem. However, they can also be used for projects outside Autoware, or even outside autonomous driving. Note: due to its use in autoware, even if it is possible to use the sensor calibration tools independently, due to some light dependencies, the core of autoware still needs to be downloaded, even if it is not compiled.

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

With a similar motivation than the previous Section, in some cases a native build is not possible or convenient. In order to accommodate to those situations, we also offer the sensor calibration tools as a docker image:

```bash
# Build
DOCKER_BUILDKIT=1 docker build --ssh default -t ghcr.io/tier4/calibration-tools:2.0 -f docker/Dockerfile ..

# Run - Modify if needed
docker run --gpus all --net=host -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --device=/dev/dri:/dev/dri -it ghcr.io/tier4/calibration-tools:2.0 /bin/bash
```

## Implemented tools

### Extrinsic calibration tools

| Name                                | Sensors calibrated | Feature type                | Calibration type   | Documentation                              | Tutorial                                           |
| ----------------------------------- | ------------------ | --------------------------- | ------------------ | ------------------------------------------ | -------------------------------------------------- |
| ground plane calibrator             | base-lidar         | ground                      | roll, pitch, z     | [Link](ground_plane_calibrator/README.md)  | Link                                               |
| interactive camera-lidar calibrator | camera-lidar       | manual correspondences      | full pose          | Link                                       | Link                                               |
| lidar-lidar 2d calibrator           | lidar-lidar        | natural features            | x, y, yaw          | Link                                       | Link                                               |
| mapping calibrator (lidar-lidar)    | lidar-lidar        | natural features            | full pose          | Lik                                        | Link                                               |
| mapping calibrator (base-lidar)     | base-lidar         | natural features and ground | roll, pitch, and z | Link                                       | Link                                               |
| marker radar-lidar calibrator       | radar-lidar        | marker                      | x, y, yaw          | Link                                       | Link                                               |
| tag-based pnp calibrator            | camera-lidar       | marker                      | full pose          | Link                                       | Link                                               |
| tag-based SfM calibrator            | camera-lidar-base  | marker                      | full pose          | [Link](tag_based_sfm_calibrator/README.md) | [Link](docs/tutorials/tag_based_sfm_calibrator.md) |

### Intrinsic calibration tools

| Name                         | Sensors calibrated | Feature type       | Calibration type    | Demo |
| ---------------------------- | ------------------ | ------------------ | ------------------- | ---- |
| camera intrinsics calibrator | camera intrinsics  | calibration boards | OpenCV camera model | Link |

## Design

## Integration

### sensor
