# EM Tracking Project

This repository contains the code for the `em_tracking` project.

## Nodes

### Pickup Node

This node is used to continuously read from the DAQ card. Use with `ros2 launch em_tracking pickup.xml`.

| Parameter Name  | Right columns |
| ------------- |:-------------:|
| sample_rate     | DAQ sample rate in Hz (default: 200kHz) |
| samples_per_channel      | samples per DAQ channel (default: 2000) |
