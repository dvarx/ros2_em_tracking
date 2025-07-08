#!/usr/bin/python3
"""
this script reads a single `/pickup_node/voltage_frames` message and stores it
`/pickup_node/voltage_frames` contains a single array of shape (5000,4) containing the voltages sampled
in all four channels
"""

from __future__ import absolute_import, division, print_function
from builtins import *  # @UnusedWildImport
from std_msgs.msg import Float64MultiArray
import rclpy
import numpy as np
import time
from std_msgs.msg import Float32MultiArray
from magnetic_tracking import extract_windowed_pdft, plot_dataframe_dft
import matplotlib.pyplot as plt
from mdriver.srv import StateTransition
from mdriver_py.mdriver_node import MDriverNode

import os
databasepath=os.getenv("LOCALDATABASEPATH")
if(databasepath==None):
    print("Environment variable `LOCALDATABASEPATH` not set, aborting")
    exit(1)

no_measurements=128
meas_counter=0
datalist=[]
NSAMPLE=2000

def voltage_frames_cb(msg):
    global meas_counter
    print(meas_counter)
    if meas_counter<=no_measurements:
        #data[sampleno,channelno]
        d=np.array(msg.data).reshape(NSAMPLE,4)
        datalist.append(d)
        meas_counter=meas_counter+1
        print(meas_counter)

if __name__ == '__main__':
    #initialize the smaract stage and drive it around
    rclpy.init()
    drivernode=MDriverNode()
    drivernode.enable_driver()
    time.sleep(2)
    drivernode.run_regular_driver()
    time.sleep(2)

    #measure frames
    pos_sub=drivernode.create_subscription(Float32MultiArray,"/pickup_node/voltage_frames",voltage_frames_cb,10)
    while(meas_counter<no_measurements):
        time.sleep(1e-3)
        rclpy.spin_once(drivernode)
    print("Finished measurement")
    data=np.array(datalist)
    N=data.shape[1]
    datas=data
    data=data[0,:,:]
    fsample=200e3
    T=N/fsample
    df=1/T

    data_dfts=np.zeros(datas.shape,dtype="complex64")
    for n in range(0,datas.shape[0]):
        data_dfts[n,:,:]=extract_windowed_pdft(datas[n,:,:],"blackman")

    #plot the phasors
    ks=[70,81,98]
    fig,ax=plt.subplots(3,1)
    for freqno in range(0,3):
        ax[freqno].scatter(np.real(data_dfts[:,ks[freqno],0]),np.imag(data_dfts[:,ks[freqno],0]),s=1)
        ax[freqno].scatter(np.real(data_dfts[:,ks[freqno],1]),np.imag(data_dfts[:,ks[freqno],1]),s=1)
        ax[freqno].scatter(np.real(data_dfts[:,ks[freqno],2]),np.imag(data_dfts[:,ks[freqno],2]),s=1)
    plt.show()
    pass

    from numpy.linalg import norm
    from math import log10
    for k in ks:
        snr=norm(data_dfts[:,:,0:3],axis=2)[:,k].mean()/norm(data_dfts[:,:,0:3],axis=2)[:,k].std()
        print("SNR at k=%d : %fdB"%(k,20*log10(snr)))


    yticks=[10**(-n) for n in range(0,7)]
    plt.figure(1)
    plt.subplot(411)
    plt.semilogy(1e-3*df*np.arange(0,N),np.sqrt((np.abs(data_dfts[:,:,0])**2).mean(axis=0)),color="red")
    plt.yticks(yticks)
    plt.ylabel("$V_x\quad[V_{rms}]$")
    plt.grid()
    plt.subplot(412)
    plt.semilogy(1e-3*df*np.arange(0,N),np.sqrt((np.abs(data_dfts[:,:,1])**2).mean(axis=0)),color="green")
    plt.yticks(yticks)
    plt.ylabel("$V_y\quad[V_{rms}]$")
    plt.grid()
    plt.subplot(413)
    plt.semilogy(1e-3*df*np.arange(0,N),np.sqrt((np.abs(data_dfts[:,:,2])**2).mean(axis=0)),color="blue")
    plt.yticks(yticks)
    plt.ylabel("$V_z\quad[V_{rms}]$")
    plt.grid()
    plt.subplot(414)
    plt.semilogy(1e-3*df*np.arange(0,N),np.sqrt((np.abs(data_dfts[:,:,3])**2).mean(axis=0)),color="gray")
    plt.yticks(yticks)
    plt.ylabel("$V_{ref}\quad[V_{rms}]$")
    plt.grid()
    plt.suptitle("5000 sample DFT sampled at 100kHz (20Hz bins)")
    plt.gcf().set_size_inches((8,6))
    #plt.plot(data[:,3],color="gray")
    plt.xlabel("Frequency [kHz]")
    plt.tight_layout()
    plot_dataframe_dft(data,N=2000)
    plt.show()

    #store the measurement data
    import pickle as pkl

    from datetime import datetime
    nowstr=datetime.today().strftime('%Y-%m-%d-%H-%M-%S')
    with open(databasepath+"/em_tracking/measurement_data_%s.pkl"%(nowstr),"wb") as fptr:
        pkl.dump(datas,fptr)
    drivernode.stop_driver()
    #change the file permission to read only
    print("stored data")
    exit(0)