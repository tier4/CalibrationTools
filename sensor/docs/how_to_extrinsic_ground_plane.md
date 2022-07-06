# Extrinsic ground-plane calibration (base-lidar)

## 1. Prepare the calibration environment

This calibration method is based on the premise that the vehicle is placed on a regular surface so that it forms a plane, with as few objects as possible on the floor. The lidar subject to calibration must observe as much of the ground plane as possible for this method to obtain a accurate calibration.

## 2. Capture rosbag

Capture a rosbag including lidar subject to calibration. The duration is arbitrary, but at least a few seconds are required.

<details><summary>ROSBAG Example</summary>
<p>

```sh
ros2 bag info f8c99b4f-a605-4178-97ef-58bfc39eabe9_2022-02-07-14-23-31_0.db3

[INFO] [1632971360.501197002] [rosbag2_storage]: Opened database 'f8c99b4f-a605-4178-97ef-58bfc39eabe9_2022-02-07-14-23-31_0.db3' for READ_ONLY.

Files:             f8c99b4f-a605-4178-97ef-58bfc39eabe9_2022-02-07-14-23-31_0.db3
Bag size:          3.7 GiB
Storage id:        sqlite3
Duration:          51.59s
Start:             Feb  7 2022 14:23:32.345 (1644211412.345)
End:               Feb  7 2022 14:24:23.404 (1644211463.404)
Messages:          252554
Topic information: Topic: /sensing/lidar/right_upper/pandar_packets | Type: pandar_msgs/msg/PandarScan | Count: 502 | Serialization Format: cdr
```

</p>
</details>

## 3. Launch Calibration Tools

To launch the extrinsic ground-plane calibration tool, use the following command (on terminal 1).

```sh
ros2 launch extrinsic_calibration_manager calibration.launch.xml \
  mode:=ground_plane sensor_model:=<sensor_model> vehicle_model:=<vehicle_model> vehicle_id:=<vehicle_id>
```

For example,

```sh
ros2 launch extrinsic_calibration_manager calibration.launch.xml \
  mode:=ground_plane sensor_model:=aip_x1 vehicle_model:=lexus vehicle_id:=my_awesome_vehicle
```

The, play the recorded calibration rosbag (on terminal 2).

```sh
ros2 bag play <rosbag_path> --clock -l -r 0.2 \
  --remap /tf:=/null/tf /tf_static:=/null/tf_static
```

The previous commands are nedded when performing the calibration on `rosbag` data. However, the calibration can also be performed on the actual vehicles, on which case the commands needs to be modified, since the `logging_simulator` and related functionalities are not needed. To to this, add the `logging_simulator:=false`

For example,

```sh
ros2 launch extrinsic_calibration_manager calibration.launch.xml \
  mode:=ground_plane sensor_model:=aip_x1 vehicle_model:=lexus vehicle_id:=my_awesome_vehicle logging_simulator:=false
```

Once the calibration process ends, the results are written in the `$HOME` folder. In order for Autoware to use them, they must be placed in the corresponding `individual_params` package's `config` folder. For example:

```sh
$HOME/autoware/src/autoware/individual_params/individual_params/config/default/aip_x1
```

## 4. Base-lidar calibration

The base_link is a frame that lies on the floor on the rear axis of the vehicle. Due to its location, its calibration can prove to be very cumbersome involving precise measurements and conditions, or requiring considerable infrastructure.

This calibration does not attempt to solve the full base-lidar calibration process (calculation of the x, y, z, roll, pitch, and yaw values) but by using the assumption that the `base_link` lies on the plane formed by the ground, we can estimate the ground plane seen by the laser, and obtain the z, roll, and pitch values (since this calibration method does not estimate x, y, nor yaw, it uses the initial calibration as output for those values).

## 5. Base-lidar calibration Process

Upon launching the tool, the calibration process is automatic, and its result can be visualized through the provided rviz profile.

In Fig 1, a normal pre-calibration scenario can be observed, where the XY-plane formed by the `base_link` frame is not correctly aligned with the points that lie on the floor. In this case, independent from the x, y, and yaw values from the initial calibration, the x, roll, and pitch values are incorrect and are the target of this method. In contrast, if the calibration is correct, in terms of z, roll, and pitch, the XY plane is aligned perfectly as shown in Figure 7.

| ![tagbased-1.jpg](images/base-lidar/initial_calibration.png) | ![tagbased-2.jpg](images/base-lidar/final_calibration.png) |
| :----------------------------------------------------------: | :--------------------------------------------------------: |
|                  Fig 6. Initial calibration                  |                Fig 7. Automatic calibration                |

To confirm the calibration accuracy, the user can refer to several frames, and observe how the grid displayed on the XY plane of the respective frame compares against the point cloud. Following are the relevant frames and their description.

- `base_link`: This frame should lie on the floor and be aligned with respect to the car. If the launcher parameter `broacast_calibration_tf` is set to `True`, it displays the calibration result, whereas if the parameter is set to `False` it displays the frame before the calibration process. It defaults to `True`
- `initial_base_link`: This frame represents the original values before the calibration process. It is used to easily switch between the before/after calibration views in order to evaluate how the calibration changed.
- `ground_plane`: This frame lies on the detected floor, and from this frame, the `roll`, `pitch`, and `z` values are extracted for the output values. Conversely, its `x`, `y`, and `yaw` do not bear relation with the calibration output (since these values can not be estimated using this method).

After selecting the frame to evaluate, the user must set the camera in a convenient place to check if the point cloud lies on the respective frame's XY plane. To do this, it is recommended to put the Rviz camera in `FPS` mode and set both the `z` and `pitch` values to `0` (this is done by default in the provided rviz profile) and set the `x`, `y`, and `yaw` values to the selected evaluation position and orientation. A good location corresponds to the origin of the frame while varying the `yaw` since it allows evaluating the calibration from the perspective of the robot.

Another way to evaluate the calibration is to check which points of the point cloud are considered inliers in the model. This calibration estimated the ground plane via RANSAC, in which a model is estimated and those points that agree with the model are considered inliers. As such, if the estimated model is suspected to be incorrect, by checking the estimated inliers the user can get insight into whether there is a problem with the estimation or with the environment itself.
