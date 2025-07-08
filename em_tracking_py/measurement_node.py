#!/usr/bin/python3
"""
this script reads N `/pickup_node/voltage_frames` messages and stores it
as a list of (2000,4) arrays
"""

from __future__ import absolute_import, division, print_function
from builtins import *  # @UnusedWildImport
from std_msgs.msg import Float32MultiArray
import rclpy
from rclpy.node import Node
import numpy as np
import os
from stat import S_IREAD, S_IRGRP, S_IROTH
import pickle as pkl
import os
databasepath=None

no_measurements=256
meas_counter=0
datas=[]

def voltage_frames_cb(msg):
    global meas_counter, no_measurements
    #data[sampleno,channelno]
    data=np.array(msg.data).reshape(2000,4)
    datas.append(data)
    meas_counter=meas_counter+1
    print(meas_counter)
    if meas_counter==no_measurements:
        data=np.array(datas)
        #store the measurement data
        from datetime import datetime
        nowstr=datetime.today().strftime('%Y-%m-%d-%H-%M-%S')
        with open(databasepath+"/measurement_data_%s.pkl"%(nowstr),"wb") as fptr:
            pkl.dump(data,fptr)
        #change the file permission to read only
        os.chmod(databasepath+"/measurement_data_%s.pkl"%(nowstr),S_IREAD|S_IRGRP|S_IROTH)
        print("stored data")
        exit(0)

if __name__ == '__main__':
    #initialize the smaract stage and drive it around
    rclpy.init()
    node=Node("measurement_node")
    node.declare_parameter("no_measurements",rclpy.Parameter.Type.INTEGER)
    no_measurements_param = rclpy.parameter.Parameter(
        'no_measurements',
        rclpy.Parameter.Type.INTEGER,
        128
    )
    node.set_parameters([no_measurements_param])
    no_measurements=node.get_parameter("no_measurements").value
    try:
        databasepath=os.getenv("LOCALDATABASEPATH")+"/em_tracking"
    except:
        node.get_logger().error("Could not find path to local database, set environment variable `LOCALDATABASEPATH`")
        node.get_logger().error("Terminating node")
        exit(1)
    frame_subscriber=node.create_subscription(Float32MultiArray,"/pickup_node/voltage_frames",voltage_frames_cb,10)
    rclpy.spin(node)