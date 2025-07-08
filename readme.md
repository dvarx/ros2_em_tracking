# EM Tracking Project

This repository contains the code for the `em_tracking` project.

## Launch Files

| Launch File Name  | Description |
| ------------- |:-------------:|
| `signal_test.xml`     | This launch file runs the `mdriver_node` and the `pickup_node` and acquires a set of test measurements |

## Nodes

### Pickup Node

This node is used to continuously read from the `USB-1808GX` DAQ card. Use with `ros2 launch em_tracking pickup.xml`.

| Parameter Name  | Right columns |
| ------------- |:-------------:|
| sample_rate     | DAQ sample rate in Hz (default: 200kHz) |
| samples_per_channel      | samples per DAQ channel (default: 2000) |

### Measurement Node

This node is used to take `N` voltage frame measurements for postprocessing.
```bash
ros2 run em_tracking measurement_node.py --ros-args -p no_measurements:=64
```

## Environment Variables

- `LOCALDATABASEDIR` : Local database directory, default storage location for measurements 