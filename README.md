# calibration_tools

Calibration tools used for autonomous driving

## Requirement

- Ubuntu 20.04
- ROS 2 Galactic

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

[README](https://github.com/tier4/calibration_tools.iv/blob/tier4/universe/sensor/README.md)

### localization - deviation estimation tools

Estimate parameters of sensors used for dead reckoning (IMU and odometry) for a better localization performance

[README](https://github.com/tier4/calibration_tools.iv/blob/tier4/universe/localization/deviation_estimation_tools/ReadMe.md)

### control - vehicle cmd analyzer

Visualization and analysis tools for the control outputs from Autoware

[README](https://github.com/tier4/calibration_tools.iv/blob/tier4/universe/control/vehicle_cmd_analyzer/README.md)

### vehicle - time delay estimator

Calibration tool to fix the delay of the commands to the vehicle

[README](https://github.com/tier4/calibration_tools.iv/blob/tier4/universe/vehicle/time_delay_estimator/README.md)

### system - tunable static tf broadcaster

GUI to modify the parameters of generic TFs.

[README](https://github.com/tier4/calibration_tools.iv/blob/tier4/universe/system/tunable_static_tf_broadcaster/README.md)
