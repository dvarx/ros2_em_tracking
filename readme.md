# MDriver ROS2 Package for EM-Tracking

This folder contains the `MDriver` ROS2 package that is used to control the TNBeMNS ("Mini-Navion" System) for use with the electromagnetic tracking setup.

* Compatible hardware version is `MDriver v1.0` ([hardware repository](https://github.com/dvarx/mdriver_hw))
* Compatibale firmware version is ij repository: ([firmware repo](https://github.com/dvarx/mdriver_emtracking_fw.git))

![MDriver hardware v1.0](img/emtracking_driver_hw.jpg)

## Services

| Service Name        | Data Type | Description                               |
| ------------------- | --------- | ----------------------------------------- |
| `/mdriver/enable`      | `bool[] enable`    | if `enable[i]==true`, then channel i will transition from `READY` to `BUCK_ENABLED` |
| `/mdriver/run_regular`        | `bool[] enable`    | if `enable[i]==true` then channel i will transition from `BUCK_ENABLED` to `RUN_REGULAR`|
| `/mdriver/run_resonant` | `bool[] enable`    | if `enable[i]==true` then channel i will transition from `BUCK_ENABLED` to `RUN_RESONANT` |
| `/mdriver/stop` | `bool[] enable`    | if `enable[i]==true` then channel i will transition from its current state to `READY` |

## Topics

| Topic Name               | Data Type    | Description                                                  |
| ------------------------ | ------------ | ------------------------------------------------------------ |
| `/mdriver/des_currents_reg` | `float64[]` | desired currents for the 6 coils in [A] |
| `/mdriver/des_duties` | `float64[]` | desired duty cycles for the buck converter stage in interval [0.1,0.9] |
| `/mdriver/des_freqs` | `uint32[]` | desired frequencies in resonant mode in [mHz] |

## Parameters

| Node | Parameter | Type | Description |
|---|---|---|---|
| `mdriver_node` | `use_prefilter` | `bool` | If set, the current commands will be first-order low pass filtered using `prefilter_tau` |
| `mdriver_node` | `prefilter_tau` | `double` | Time constant of the first order prefilter |
| `mdriver_node` | `status_msg_downsample` | `int` | The node will publish every n-th status report it receives from the `mdriver`, where n is the value of this parameter |
| `mdriver_node_test` | `rectangular_current` | `bool` | If set, a rectangular current will be tracked instead of a sinusoidal one |
| `mdriver_node_test` | `amplitude` | `double` | Amplitude to be tracked in [A] |
| `mdriver_node_test` | `frequency` | `double` | Signal frequency to be tracked in [Hz] |


## Test Nodes

- `mdriver_gui.py` (`ros2 run mdriver mdriver_gui.py`) runs a simple GUI that can be used to turn the system on and off, apply currents and display currents.
- `mdriver_field_gui.py` (`ros2 run mdriver mdriver_field_gui.py`) runs a simple GUI that can be used to turn the system on and off and apply magnetic fields.
- `mdriver_fieldgrad_gui.py` (`ros2 run mdriver mdriver_fieldgrad_gui.py`) runs a simple GUI that can be used to apply field and gradients.
- `mdriver_node_test` is a test node which puts the driver in the `RUN_REGULAR` mode and applied a sinusoidal output voltage on channel 1. The node demonstrated basic control of the driver. Run using `ros2 launch mdriver_test.xml`.
- `mdriver_node_test.py` defines a python wrapper class for accessing the `MDriver`

## Terminal testing
To test the `mdriver` hardware, the following ROS2 terminal commands can be used

    ros2 run mdriver mdriver_node
    ros2 service call /mdriver/enable mdriver/srv/StateTransition "{enable: [true, true, true]}"
    ros2 service call /mdriver/run_regular mdriver/srv/StateTransition "{enable: [true, true, true]}"

    ros2 topic pub /mdriver/des_currents_reg std_msgs/msg/Float32MultiArray "{layout: {}, data: [1.0, 0.0, 0.0]}" -1

    ros2 service call /mdriver/stop mdriver/srv/StateTransition "{enable: [true, true, true]}"