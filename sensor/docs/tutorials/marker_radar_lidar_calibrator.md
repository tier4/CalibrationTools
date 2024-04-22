# Marker radar lidar calibrator

Commands for running the tools (make sure to source the setup.bash before launching.)

Terminal 1: Launch autoware

```sh
ros2 launch autoware_launch logging_simulator.launch.xml map_path:=/home/yihsiangfang/autoware_map/sample-rosbag vehicle_model:=j6_gen1 sensor_model:=aip_x2 vehicle_id:=j6_gen1_01 rviz:=false localization:=false perception:=true map:=false control:=false planning:=false
```

Terminal 2: Launch the calibration tool

```sh
ros2 run sensor_calibration_manager sensor_calibration_manager
```

Change the parameters if needed, and make sure that you select the correct radar name.

Press the calibrate button to start the tool and then you can start to play the bag

Terminal 3: Play bag

```sh
ros2 bag play name_of_rosbag --clock --remap /tf:=/tf_old /tf_static:=/tf_static_old -r 0.2
UI, Rviz and Metric plotter
```

After the lidar pointcloud shows on the rviz.

First press the extract background model to extract the background

Afterward, when the lidar and the radar detection show up, press the add lidar-radar pair to add them for calibration,

After you add more than three lidar-radar pairs, the metric plotter will show the average calibration error. After the pairs are more than four, the cross-validation error will also show on the plotter with an additional std error.

During the calibration, if you add some pairs that are not stable or mismatched, you can click the delete the previous pair to delete them

Finally, when the cross-validation error is converged, you can press the send calibration to stop the calibration and then click the save calibration to save the calibration result in yaml
