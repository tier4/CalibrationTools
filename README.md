# calibration_tools

Calibration tools used for autonomous driving

## Requirement

- Ubuntu22.04
- Ros Humble

## Installation procedures

After installing [autoware](https://github.com/tier4/autoware) (please see [source-installation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/) page), execute the following commands:

```bash
cd autoware
wget https://raw.githubusercontent.com/tier4/CalibrationTools/tier4/universe/calibration_tools.repos
vcs import src < calibration_tools.repos
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Implemented calibration tools

### sensor

We provide calibration tool for sensor pairs like LiDAR - LiDAR, LiDAR - Camera, etc.

[README](sensor/README.md)

### system - tunable static tf broadcaster

GUI to modify the parameters of generic TFs.

[README](system/tunable_static_tf_broadcaster/README.md)
