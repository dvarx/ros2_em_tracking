#!/usr/bin/python3
"""
this script reads a single `/pickup_node/voltage_frames` message and stores it
`/pickup_node/voltage_frames` contains a single array of shape (5000,4) containing the voltages sampled
in all four channels
"""

from __future__ import absolute_import, division, print_function
from builtins import *  # @UnusedWildImport
from std_msgs.msg import Float32MultiArray
import rclpy
import numpy as np
import pickle as pkl
from rclpy.node import Node
import os
from stat import S_IREAD, S_IRGRP, S_IROTH
import os
databasepath=os.getenv("LOCALDATABASEPATH")
if(databasepath==None):
    print("Environment variable `LOCALDATABASEPATH` not set, aborting")
    exit(1)

no_measurements=128
no_positions=8
record=False
meas_counter=0
datas=[]

def voltage_frames_cb(msg):
    global meas_counter, record
    if not record or meas_counter>=no_measurements:
        return
    #data[sampleno,channelno]
    data=np.array(msg.data).reshape(2000,4)
    datas.append(data)
    meas_counter=meas_counter+1
    print(meas_counter)

if __name__ == '__main__':
    #initialize the smaract stage and drive it around
    rclpy.init()
    node=Node("measurement_node_calibration")

    frame_subscriber=node.create_subscription(Float32MultiArray,"/pickup_node/voltage_frames",voltage_frames_cb,10)

    for phino in range(0,no_positions):
        input("click to take recording for position number %d"%(phino))
        meas_counter=0
        datas=[]
        record=True
        while(meas_counter<no_measurements):
            rclpy.spin_once(node)
        data=np.array(datas)
        #store the measurement data
        with open(databasepath+"/em_tracking/measurements/refsensor_Omega3_%d.pkl"%(phino+1),"wb") as fptr:
            pkl.dump(data,fptr)
        #change file permission to read only
        os.chmod(databasepath+"/em_tracking/measurements/refsensor_Omega3_%d.pkl"%(phino+1),S_IREAD|S_IRGRP|S_IROTH)

