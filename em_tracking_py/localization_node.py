#!/usr/bin/python3
"""
this script reads a single `/pickup_node/voltage_frames` message and stores it
`/pickup_node/voltage_frames` contains a single array of shape (5000,4) containing the voltages sampled
in all four channels
"""
from __future__ import absolute_import, division, print_function
from builtins import *  # @UnusedWildImport
import rclpy.logging
from std_msgs.msg import Float32MultiArray
from mdriver.msg import Status
import geometry_msgs.msg
import rclpy
import tf2_ros
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import PointStamped, Point
from rclpy.node import Node
import time
from mdriver_py.mdriver_node import MDriverNode
import numpy as np
import pickle as pkl
from scipy.optimize import least_squares
import os
from stat import S_IREAD, S_IRGRP, S_IROTH
from magnetic_tracking import pickup_coils, getvoltphasormat, findrotmat
import time
import os
databasepath=os.getenv("LOCALDATABASEPATH")
if(databasepath==None):
    print("Environment variable `LOCALDATABASEPATH` not set, aborting")
    exit(1)

ks=[70,81,98]
last_pos=(4,4,4)
sensorsigpowercalfile=databasepath+"/em_tracking/measurements/calibrations/sens13_sigpowcal.pcal"
sensorphasorcalfile=databasepath+"/em_tracking/measurements/calibrations/refsensor_phasorcal.pcal" 
fieldmapcalfile=databasepath+"/em_tracking/measurements/calibrations/fieldmap_16cm16cm16cm_60Hz_refsensor.pkl"

#calibration data for pickup coil
sensor=pickup_coils(sensorsigpowercalfile,sensorphasorcalfile,ks)
#fieldmaps for signal powers and phasor matrices
with open(fieldmapcalfile,"rb") as fptr:
    data=pkl.load(fptr)
sigpowerinterp=data["sigpowerinterpolator"]
phasormatinterp=data["phasormatinterp"]
#define localization method
def localize(sigpowers,initial_gues=(4,4,4)):
    residuals=lambda pos : 1e6*(sigpowerinterp(pos)-sigpowers).flatten()
    res=least_squares(residuals,(4,4,4),bounds=([0,0,0],[8,8,8]),ftol=1e-5,gtol=1e-5)
    return res

#transform matrix used for aligning catheter to emns frame, use __comp_rot_mat for computation
trans=np.eye(3)#np.array([[-1,0,0],[0,1,0],[0,0,1]])

#low pass filter related
low_pass_pos_filter=True
xs=0
ys=0
zs=0
#xs=alpha*xs+(1-alpha)*x
alpha=0.0

use_ref_sensor=True
record_ref_powers=False
reference_powers_list=[]
reference_powers=np.zeros(3)
refpowers_counter=0
def voltage_frames_cb(msg):
    global xs,ys,zs, last_pos, loc_node
    global meas_counter,corr_factors,refpowers_counter, reference_powers
    #data[sampleno,channelno]
    data=np.array(msg.data).reshape(2000,4)
    #extract signal powers
    (voltphasormat,refsensorpows)=getvoltphasormat(data,ks,compute_ref_sensor=True)
    if(record_ref_powers):
        reference_powers_list.append(refsensorpows)
        refpowers_counter=refpowers_counter+1
    signal_powers=sensor.getsignalpowers(voltphasormat)

    #use the reference sensor measurements
    #rospy.loginfo("%f"%(reference_powers[0]/np.abs(refsensorpows[0])))
    if(use_ref_sensor):
        signal_powers[0]=signal_powers[0]*reference_powers[0]/np.abs(refsensorpows[0])
        signal_powers[1]=signal_powers[1]*reference_powers[1]/np.abs(refsensorpows[1])
        signal_powers[2]=signal_powers[2]*reference_powers[2]/np.abs(refsensorpows[2])
    #estimate the position
    res=localize(signal_powers,initial_gues=last_pos)
    #estimate the orientation
    phasormatref=phasormatinterp(res.x)[0,:,:]
    last_pos=res.x
    phasormat=sensor.getphasormat(voltphasormat)
    rotmat=trans.dot(findrotmat(phasormatref,phasormat,method="Procrustes").transpose())
    rotation=Rotation.from_matrix(rotmat)
    #filter the localized points
    pointmsgstamped=PointStamped()
    pointmsg=Point()
    powersmsg=Point()
    tfmsg = geometry_msgs.msg.TransformStamped()
    powersmsg.x=signal_powers[0]
    powersmsg.y=signal_powers[1]
    powersmsg.z=signal_powers[2]
    sigpowers_pub.publish(powersmsg)
    if low_pass_pos_filter:
        xs=alpha*xs+(1-alpha)*res.x[0]
        ys=alpha*ys+(1-alpha)*res.x[1]
        zs=alpha*zs+(1-alpha)*res.x[2]
        pointmsgstamped.point.x=-(xs*0.02-0.08)
        pointmsgstamped.point.y=-(ys*0.02-0.08)
        pointmsgstamped.point.z=-(zs*0.02-0.08)
    else:
        pointmsgstamped.point.x=-(res.x[0]*0.02-0.08)
        pointmsgstamped.point.y=-(res.x[1]*0.02-0.08)
        pointmsgstamped.point.z=-(res.x[2]*0.02-0.08)
    #publish the localized point
    pointmsgstamped.header.frame_id = 'emns'
    pointmsg=pointmsgstamped.point
    pos_pub.publish(pointmsgstamped)
    pos_pub2.publish(pointmsg)
    #publish the transform (note, need to give the rotation matrix in homogenous coordinates, i.e. a 4x4 matrix is required)
    tfmsg.transform.translation.x=pointmsgstamped.point.x
    tfmsg.transform.translation.y=pointmsgstamped.point.y
    tfmsg.transform.translation.z=pointmsgstamped.point.z
    quat=rotation.as_quat().flatten()
    tfmsg.transform.rotation.x=quat[0]
    tfmsg.transform.rotation.y=quat[1]
    tfmsg.transform.rotation.z=quat[2]
    tfmsg.transform.rotation.w=quat[3]
    tfmsg.header.frame_id="pickups"
    tfmsg.child_frame_id = "emns"
    #tfmsg.header.stamp=loc_node.get_clock().now()
    transform_pub.sendTransform(tfmsg)

loc_node=None
if __name__ == '__main__':

    # initialize the smaract stage and drive it around
    rclpy.init()
    loc_node=Node("localization_node")
    pos_sub=loc_node.create_subscription(Float32MultiArray,"/pickup_node/voltage_frames",voltage_frames_cb,10)
    pos_pub=loc_node.create_publisher(PointStamped,"/em_tracking/position",10)
    pos_pub2=loc_node.create_publisher(Point,"/em_tracking/position2",10)
    sigpowers_pub=loc_node.create_publisher(Point,"/em_tracking/sigpowers",10)
    transform_pub=tf2_ros.TransformBroadcaster(loc_node)
    
    #put the driver into the enable state
    time.sleep(1)
    #get handle for STOP service
    driver_node=MDriverNode()
    driver_node.enable_driver()
    time.sleep(1)
    driver_node.run_regular_driver()
    time.sleep(1)

    #measure the reference signal powers
    loc_node.get_logger().info("Measuring reference powers")
    record_ref_powers=True
    while(refpowers_counter<128):
        rclpy.spin_once(loc_node)
    record_ref_powers=False
    loc_node.get_logger().info("Acquired reference values for reference sensor")
    reference_powers_list=np.abs(np.array(reference_powers_list))
    reference_powers=reference_powers_list.mean(axis=0)
    rel_stds=reference_powers_list.std(axis=0)/reference_powers_list.mean(axis=0)
    loc_node.get_logger().info("Relative standard deviations: %f%% , %f%% , %f%%"%(100*rel_stds[0],100*rel_stds[1],100*rel_stds[2]))
    
    #spin forever
    rclpy.spin(loc_node)