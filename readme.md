# EM Tracking Project

## Minimal startup procedure
### Turning the DAQ setup on
* Enable the battery power to the board by switching the power switch
* Power cycle the amplifier circuit using the red jumpers
### Turning the power supplies on
* [0] Power on both PSUs using the `POWER` button
* [1] Enable serial connection on PSU II (`SER` should light up on PSU display)
* [2] Enable output power on PSU I
* [3] Enable output power on PSU II

### Turning the smaract stage on
* Flip the power switch on the controller for the stage

### Starting the required ROS nodes
#### With launch script
To start all required ROS nodes and services, use the `roslaunch` command which also takes care of starting `roscore` if it is not already running:s

```bash
$ roslaunch em_tracking start_setup.launch
```

To stop all running ROS nodes and services initiated by `roslaunch`, press `Ctrl+C` in the terminal where `roslaunch` is running.

#### Manual start
```bash
# run the driver node
$ rosrun tnb_mns_driver tnb_mns_driver_node     

# enable power output of the driver
$ rosservice call /tnb_mns_driver/enable [True,True,True,True,True,True]
$ rosservice call /tnb_mns_driver/run_regular [True,True,True,True,True,True]

# start reading voltage values from pickup coil
$ LD_LIBRARY_PATH=/opt/ros/noetic/lib:/opt/pylon5/lib64
$ rosrun em_tracking pickup_node  

# start stage
$ LD_LIBRARY_PATH=/local/home/fschnitzler/tesla_ws/devel/lib:/opt/ros/noetic/lib:/opt/pylon5/lib64
$ rosrun smaract_ros smaract_node

# conduct your experiment

# turn off output driver power
$ rosservice call /tnb_mns_driver/stop [True,True,True,True,True,True]      
```




### Nodes
| Node Name                | Description                                                    |
|--------------------------|----------------------------------------------------------------|
| `pickup_node`            | Publishes sampled voltage frames from DAQ device.              |
| `localization_node.py`   | Subscribes to `/pickup_node/voltage_frames` and publishes `/pickup_node/est_position`. |
| `tnb_mns_driver_node`    | Handles communication with the coil driver.                    |
| `smaract_node`           | Interfaces and controls the SmarAct 3D piezo stage.            |
| `service_caller.py`      | Automatically calls ROS services to enable, run, and stop the driver as part of the system startup and shutdown processes. |
| `realtime_localization_node.py`| Estimates the position of the sensor based on the sensordata and publishes the estimated position|

In order to enable the driver, call the following services:
```
rosservice call /tnb_mns_driver/enable [True,True,True,True,True,True]
rosservice call /tnb_mns_driver/run_regular [True,True,True,True,True,True]
```
In order to disable the driver, call the following service:
```
rosservice call /tnb_mns_driver/stop [True,True,True,True,True,True]
```

### Messages & Topics
| Topic Name                  | Data Type                          | Description                                                         |
|-----------------------------|------------------------------------|---------------------------------------------------------------------|
| `/pickup_node/voltage_frames` | `std_msgs/Float32MultiArray`     | Raw samples from the pickup coils [V]. Each frame contains 4x5000 samples sampled at 100kHz. |
| `/pickup_node/est_position`  | `geometry_msgs/Pose`              | Estimated pose of the sensor within the workspace.                  |
| `/realtime_localization_node/estimated_position` | `geometry_msgs/PointStamped` | Estimated pose of the sensor within the workspace.                  |

## Installation of DAQ Hardware
### Linux Installation
Install the [MCC Universal Library](https://github.com/mccdaq/uldaq) for Linux. After building, ensure that `libusb` is installed and that no other versions of `libusb` may interfere (e.g., check `LD_LIBRARY_PATH` for other versions of `libusb`).

## Actuation influence tests
Use `roslaunch em_tracking actuation_influence.launch` to launch the actuation influence launchexperiment. Use `reference_sensor_actuation_influence.py` to post-process the data

# Mono Camera
## Mono Camera Calibration (Intrinsic Camera Parameters)
Launch the camera node
```
roslaunch em_tracking camera.launch
```
Run the following command and use the 10mm checkers board to calibrate the camera:
```
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.010 image:=/em_camera_node/image_raw camera:=/em_camera_node
```
The camera calibration data should be stored to `\config\em_camera_calib.yaml`. Note that camera calibration should be performed if the focus or the aperture of the camera is changed.
## Mono Camera Registration (Camera-In-Vicon Registration)
Perform the following steps to determine to transformation between the camera frame and the vicon frame.

1. Calibrate the vicon system using the wand.
3. Place the Vicon-APRIL tag holder in the workspace. Check that the Vicon markers are well recoginzed and adjust parameters if necessary. Reset the Vicon origin.
4. Run `roslaunch em_tracking camera_in_vicon_registration.launch`
5. Check the Camera-Vicon transform correctness by launching RViz and holding the Vicon-APRIL tag holder in the workspace.

The registration node prints the position and orientation of the camera in the vicon frame to the console in the form `x y z qx qy qz qw`. These transformation values can be passed to the `camera_in_vicon_broadcaster` static transform broadcaster in `camera.launch`. After this step has been completed, run `roslaunch em_tracking localization.launch` and check the projection of the vicon frame onto the camera image.
## Vicon-In-EMNS registration
To determine the position and orientation of the Vicon frame in the EMNS frame first record data points using
```
roslaunch em_tracking vicon_in_emns_registration.launch
```
Disable the named object tracking in the Vicon software before to trck the unnamed marker. Then post-process the acquires data using `vicon_in_emns_registration_comp.py` and store the computed transform in `camera.launch` in the `vicon_in_emns_broadcaster` static transform broadcaster.

# Sensor orientation calibration
Hold the sensor into the workspace in three orthogonal directions and record the `sensor_in_ems` transform. The use `__comp_rot_mat.py` to compute the inverse transform which can be used in `localization_node.py`

# Catheter motion experiment in free space
Drive the catheter in the middle of the workspace using the `roslaunch em_tracking localization.launch`. Then use `roslaunch em_tracking actuation_localization_experiment.launch` to apply the actuation fields and move the stage. Record the Vicon data during the experiment and record the RViz screen. Record the following data using `rosbag` and post-porcess using `analyze_rosbag_data_em_vicon_actexp.bag`.
```
rosbag record -b 0 /em_camera_node/image_raw/compressed /em_tracking/position /tnb_mns_driver/des_currents_reg /pickup_node/voltage_frames /tf /vicon/markers
```

# Camera settings 

Use the `a2A 1920 160ucBAS` with 160Hz framerate. Adjust the framerate and the ROI in the Pylon software. Reduce the bandwidth below approx. 360MBit/s. Possibly set a different pixel format in `basler_camera_node.cpp`.

# Orientation evaluation tests
`rosbag` command:
```
rosbag record -b 0 /em_tracking/position /pickup_node/voltage_frames /tf /vicon/markers
```
After recording the experiment, use `rotation_eval.py` to compare the detected orientations. Orientations are measured in the eMNS coordinate frame relative to the initial orientation. Note that a **Vicon** calibration and a **Vicon-In-EMNS** calibration is necessary.