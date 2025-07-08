#!/usr/bin/python3
from __future__ import absolute_import, division, print_function
from builtins import *  # @UnusedWildImport
from std_msgs.msg import Float32MultiArray
from mdriver.msg import Status
import rclpy
from rclpy.node import Node
import time
from mdriver_py.mdriver_node import MDriverNode
import numpy as np
import matplotlib.pyplot as plt
import pickle as pkl
from three_axis_stage import threeAxisStage
from utilities import get_zigzag_pattern
import os
from stat import S_IREAD, S_IRGRP, S_IROTH
import time
import os
databasepath=os.getenv("LOCALDATABASEPATH")
if(databasepath==None):
    print("Environment variable `LOCALDATABASEPATH` not set, aborting")
    exit(1)

points_per_channel = 2000
measure_cal_points=True
measure_ver_points=True
no_frames=32
revperm_=50000/0.313

def ros_spin_some(N,node):
    """spin N times

    Args:
        N (int): number of spins to be performed
        node (Node): ROS2 node to spin 
    """
    for i in range(0,N):
        rclpy.spin_once(node)


request_samples=True
pickup_data=None
def sampled_voltages_cb(msg):
    global request_samples, pickup_data
    if(request_samples):
        pickup_data=msg.data
        print("received samples")
        request_samples=False

drvr_comm_active=False
def drvr_cb(msg):
    global drvr_comm_active
    #stepper_node.get_logger().info("status_callback")
    #stepper_node.get_logger().info("status %d,%d,%d"%(msg.states[0],msg.states[1],msg.states[2]))
    #check if all three channels in state RUN_REGULAR
    if(msg.states[0]==3 and msg.states[1]==3 and msg.states[2]==3):
        drvr_comm_active=True

if __name__ == '__main__':
    rclpy.init()
    driver_node=MDriverNode()
    driver_node.enable_driver()
    time.sleep(1)
    driver_node.run_regular_driver()
    time.sleep(1)
    
    #initialize the stepper stage
    stepper_node=Node("em_calibration_node")
    #rospy.init_node("em_calibration_node")
    stepper_node.create_subscription(Float32MultiArray,"/pickup_node/voltage_frames",sampled_voltages_cb,10)
    stepper_node.create_subscription(Status,"/mdriver/system_state",drvr_cb,10)
    stage=threeAxisStage(revperm=revperm_,port="/dev/ttyACM0")
    time.sleep(1)
    stage.setdelayus(75)
    #drive to all positions
    Nsx=16
    Nsy=16
    Nsz=16
    Nsxver=Nsx-1
    Nsyver=Nsy-1
    Nszver=Nsz-1
    Nsegcal=Nsz
    Nsegver=Nszver
    (positions,position_indices)=get_zigzag_pattern(Nsx,Nsy,Nsz,-15e-2,15e-2,-15e-2,15e-2,-15e-2,15e-2)
    (positions_ver,position_indices_ver)=get_zigzag_pattern(Nsxver,Nsxver,Nsxver,-14e-2,14e-2,-14e-2,14e-2,-14e-2,14e-2)
    no_calpoints_per_seg=int(positions.shape[1]/Nsegcal)
    no_verpoints_per_seg=int(positions_ver.shape[1]/Nsegver)
    
    #illustrate calibration and verification grid
    fig=plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.scatter(1e3*positions[0,:],1e3*positions[1,:],1e3*positions[2,:],color="black")
    ax.scatter(1e3*positions_ver[0,:],1e3*positions_ver[1,:],1e3*positions_ver[2,:],color="red")
    plt.show()

    def store_data(datadict,name):
        #store the measurement data
        with open(databasepath+"/em_tracking/measurement_data_%s.pkl"%(name),"wb") as fptr:
            pkl.dump(datadict,fptr)
        #change file permission to read only
        os.chmod(databasepath+"/em_tracking/measurement_data_%s.pkl"%(name),S_IREAD|S_IRGRP|S_IROTH)
    def move_to_position(point):
        stepper_node.get_logger().info("moving to position %s"%(str(point)))
        stage.movex(point[0])
        stage.movey(point[1])
        stage.movez(point[2])

    if measure_cal_points:
        posno=0
        #split the measurements in Nsegcal segments
        for segno in range(0,Nsegcal):
            #record the Nsegcal-th portion of the calibration data
            datas=np.zeros((no_frames,no_calpoints_per_seg,points_per_channel,3))
            posno=0
            while(posno<no_calpoints_per_seg):
                move_to_position(positions[:,segno*no_calpoints_per_seg+posno])
                time.sleep(1)
                for frameno in range(0,no_frames):
                    request_samples=True
                    #wait until a new sample has been received
                    while request_samples:
                        rclpy.spin_once(stepper_node)
                    datas[frameno,posno,:,:]=np.array(pickup_data).reshape(points_per_channel,4)[:,0:3]
                #check if communication with driver is still active
                drvr_comm_active=False
                ros_spin_some(20,stepper_node)

                if not drvr_comm_active:
                    stepper_node.get_logger().error("Communication to driver interrupted, restart driver")
                    input("Press [Enter] to continue")
                    #remeasure the last point
                    posno=posno-1
                else:
                    posno=posno+1
                    stepper_node.get_logger().info("Calibration %.2f%% done"%((segno*no_calpoints_per_seg+posno)/positions.shape[1]*100))
            #store the Nsegcal-dth calibration segment
            data_to_store={"samplefreq":100e3,"positions":positions,"position_indices":position_indices,"raw_data":datas,"NxNyNz":(Nsx,Nsy,Nsz),"indexrange":(segno*no_calpoints_per_seg,(segno+1)*no_calpoints_per_seg)}
            store_data(data_to_store,"cal_segment_%d"%(segno))

    datas_ver=np.zeros((no_frames,no_verpoints_per_seg,points_per_channel,3))
    if(measure_ver_points):
        for segno in range(0,Nsegver):
            posno=0
            #record the N-th portion of the calibration data
            datas=np.zeros((no_frames,no_verpoints_per_seg,points_per_channel,3))
            while(posno<no_verpoints_per_seg):
                move_to_position(positions_ver[:,segno*no_verpoints_per_seg+posno])
                time.sleep(1)
                for frameno in range(0,no_frames):
                    request_samples=True
                    #wait until a new sample has been received
                    while request_samples:
                        rclpy.spin_once(stepper_node)
                    datas_ver[frameno,posno,:,:]=np.array(pickup_data).reshape(points_per_channel,4)[:,0:3]
                #check if communication with driver is still active
                drvr_comm_active=False
                ros_spin_some(20,stepper_node)
                if not drvr_comm_active:
                    stepper_node.get_logger().error("Communication to driver interrupted, restart driver")
                    input("Press [Enter] to continue")
                    #remeasure the last point
                    posno=posno-1
                else:
                    posno=posno+1
                    stepper_node.get_logger().info("Verification %.2f%% done"%((posno+segno*no_verpoints_per_seg)/positions_ver.shape[1]*100))
            #store the Nsegcal-dth calibration segment
            data_to_store={"samplefreq":100e3,"positions":positions_ver,"position_indices":position_indices_ver,"raw_data":datas_ver,"NxNyNz":(Nsx,Nsy,Nsz),"indexrange":(segno*Nsegver,(segno+1)*Nsegver)}
            store_data(data_to_store,"ver_segment_%d"%(segno))

    #move back to origin
    move_to_position((0,0,0))
    #stop the driver
    driver_node.stop_driver()