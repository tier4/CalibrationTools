# Stop Accel Evaluator

The role of this node is to evaluate how smooth it is when a vehicle stops by calculating vehicle acceleration just before stopping.

## How to use

```sh
ros2 launch stop_accel_evaluator stop_accel_evaluator.launch.xml
```

Then you can see `stop_accel_evaluator/stop_accel` topic.
This topic is published only when a vehicle stops.
