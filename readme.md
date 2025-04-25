# MDriver ROS2 Package

This folder contains multiple packages that are used to control the TNBeMNS ("Mini-Navion" System). This folder contains:

* `mdriver`  package : this package implements the node `mdriver_node` to communicate with the mdriver hardware which controls the magnetic fields in the system.

## Services

| Service Name        | Data Type | Description                               |
| ------------------- | --------- | ----------------------------------------- |
| `tnb_mns_driver/Enable`      | `bool[] enable`    | if `enable[i]==true`, then channel i will transition from `READY` to `BUCK_ENABLED` |
| `tnb_mns_driver/RunRegular`        | `bool[] enable`    | if `enable[i]==true` then channel i will transition from `BUCK_ENABLED` to `RUN_REGULAR`|
| `tnb_mns_driver/RunResonant` | `bool[] enable`    | if `enable[i]==true` then channel i will transition from `BUCK_ENABLED` to `RUN_RESONANT` |
| `tnb_mns_driver/Stop` | `bool[] enable`    | if `enable[i]==true` then channel i will transition from its current state to `READY` |

## Topics

| Topic Name               | Data Type    | Description                                                  |
| ------------------------ | ------------ | ------------------------------------------------------------ |
| `/tnb_mns_driver/des_currents_reg` | `float64[]` | desired currents for the 6 coils in [A] |
| `/tnb_mns_driver/des_duties` | `float64[]` | desired duty cycles for the buck converter stage in interval [0.1,0.9] |
| `/tnb_mns_driver/des_freqs` | `uint32[]` | desired frequencies in resonant mode in [mHz] |

## Test Nodes

- `mdriver_node_test` is a test node which puts the driver in the `RUN_REGULAR` mode and applied a sinusoidal output voltage on channel 1. The node demonstrated basic control of the driver.