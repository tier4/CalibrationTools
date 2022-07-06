# Sensor Calibration Tools

## Calibration tools installation (alongside autoware)

After cloning autoware, execute the following commands:

```sh
mkdir src
vcs import src < autoware.proj.repos
vcs import src < src/autoware/calibration_tools/calibration_tools.repos
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Extrinsic Calibration

- [How to Extrinsic Manual Calibration](docs/how_to_extrinsic_manual.md)

- [How to Extrinsic Automatic Calibration](docs/how_to_extrinsic_auto.md)

- [How to Extrinsic Interactive Calibration](docs/how_to_extrinsic_interactive.md)

- [How to Extrinsic Tag-based Calibration](docs/how_to_extrinsic_tag_based.md)

- [How to Extrinsic Ground-plane Calibration](docs/how_to_extrinsic_ground_plane.md)

## Intrinsic Calibration

- [How to Camera Calibration](docs/how_to_camera.md)

- [How to Camera Calibration via camera-lidar calibration (Experimental)](docs/how_to_extrinsic_interactive.md)
