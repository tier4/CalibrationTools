# calibration_tools

Calibration tools used for autonomous driving

## Requirement

- Ubuntu22.04
- Ros Humble


## Installation procedures

After cloning pilot-auto(private repository) or [autoware](https://github.com/tier4/autoware), execute the following commands:

```bash
mkdir src
vcs import src < autoware.repos
cd src/autoware
git clone git@github.com:tier4/CalibrationTools.git
cd ../../
vcs import src < src/autoware/calibration_tools/calibration_tools.repos
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
