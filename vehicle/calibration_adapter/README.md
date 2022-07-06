# calibration_adapter

## Purpose

This package relay topic to `Float32Stamped` type of "autoware_calibration_msgs" to generalize calibration topics.

### Details

- `calibration_adapter_node_base`
  This node has general calibration topics for all vehicle interface

- `calibration_adapter`
  This node has vehicle specific or temporary topics to calibrate and this node inherit `calibration_adapter_node_base`.

## Assumptions / Known limits

TBD.
