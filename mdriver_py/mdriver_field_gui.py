#!/usr/bin/env python3

"""
This script provides a basic GUI for sending currents to the MDriver and can be used for testing and debugging purposes
"""

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QDoubleSpinBox
from PyQt5.QtCore import Qt, QThread
from mdriver_node import MDriverNode
from std_msgs.msg import Float32MultiArray
from mdriver.msg import Status
import rclpy
from math import sin, cos, pi
from numpy.linalg import pinv
import numpy as np
from magnetic_system import magnetic_system

class ROS2Thread(QThread):
    """this thread class is used to run the event loop of the ROS2 node
    """
    def __init__(self, node):
        super().__init__()
        self._node = node
        self._running = True

    def run(self):
        """this thread runs the event loop of the ROS2 node
        """
        #loops & processes ROS2 events
        rclpy.spin(self._node)

    def stop(self):
        self._running = False
        self._node.destroy_node()
        rclpy.shutdown()
        self.wait()

class MDriverFieldGUI(QWidget):
    def __init__(self,calibration_filename,Ncoils):
        """Initializes the GUI used to apply fields

        Args:
            calibration_filename (string): The name of the `pickle` calibration file containing the calibration data
            Ncoils (int): The number of coils in the eMNS
        """
        self.magsys=magnetic_system(calibration_filename,"pickle",5,5,5,Ncoils)
        self.Bact=self.magsys.getBact((0,0,0))
        self.Bactinv=pinv(self.Bact)
        self.Ncoils=Ncoils

        rclpy.init()
        self.drivernode=MDriverNode()

        self.des_currents_pub=self.drivernode.create_publisher(
            Float32MultiArray,
            "/mdriver/des_currents_reg",
            10
        )
        self.des_currents_msg=Float32MultiArray()


        self.mdriver_state_sub=self.drivernode.create_subscription(
            Status,
            "/mdriver/system_state",
            self.system_state_callback,
            10
        )

        super().__init__()
        self.setWindowTitle('Mdriver Control')
        self.setGeometry(300, 300, 300, 250)  # x, y, width, height

        # Create a vertical layout
        layout = QVBoxLayout()

        # text labels for measured currents
        self.meas_current_label_desc = QLabel('Measured Currents')
        self.meas_current_label_desc.setAlignment(Qt.AlignCenter) # Center align the text
        self.meas_currents_label = QLabel('Current Measurement')
        self.meas_currents_label.setAlignment(Qt.AlignCenter) # Center align the text
        self.meas_currents_label.setStyleSheet("font-family: Monospace;")

        # buttons for state transitions
        self.enable_button = QPushButton('Enable')
        self.run_regular_button = QPushButton('Run Regular')
        self.stop_button = QPushButton('Stop')

        # spinbox for desired magnetic field magnitude
        self.magbox=QDoubleSpinBox()
        self.magbox.setMinimum(0)
        self.magbox.setMaximum(20)
        self.magbox.setSingleStep(0.1)
        self.magbox.valueChanged.connect(self.des_field_changed)
        # spinbox for desired magnetic field azimuthal angle
        self.azimbox=QDoubleSpinBox()
        self.azimbox.setMinimum(-720)
        self.azimbox.setMaximum(720)
        self.azimbox.setSingleStep(5)
        self.azimbox.valueChanged.connect(self.des_field_changed)
        # spinbox for desired magnetic field azimuthal angle
        self.elevbox=QDoubleSpinBox()
        self.elevbox.setMinimum(-720)
        self.elevbox.setMaximum(720)
        self.elevbox.setSingleStep(5)
        self.elevbox.valueChanged.connect(self.des_field_changed)


        # Connect buttons to simple print functions (can be customized)
        self.enable_button.clicked.connect(self.enable_clicked)
        self.run_regular_button.clicked.connect(self.run_regular_clicked)
        self.stop_button.clicked.connect(self.stop_clicked)

        # Add widgets to the layout in vertical order
        layout.addWidget(self.meas_current_label_desc)
        layout.addWidget(self.meas_currents_label)
        layout.addWidget(self.enable_button)
        layout.addWidget(self.run_regular_button)
        layout.addWidget(self.stop_button)
        layout.addWidget(QLabel("Magnitude [mT]"))
        layout.addWidget(self.magbox)
        layout.addWidget(QLabel("Azimuth [deg]"))
        layout.addWidget(self.azimbox)
        layout.addWidget(QLabel("Elevation [deg]"))
        layout.addWidget(self.elevbox)

        # Set the layout for the main window
        self.setLayout(layout)

        #run the event loop of the ROS2 node in a separate thread
        self.ros2_thread=ROS2Thread(self.drivernode)
        self.ros2_thread.start()


    def enable_clicked(self):
        """put the driver in the `ENABLED` state
        """
        self.drivernode.enable_driver()

    def run_regular_clicked(self):
        """put the driver in the `RUN_REGULAR` state
        """
        self.drivernode.run_regular_driver()

    def stop_clicked(self):
        """stop the driver and put it in the `IDLE` state
        """
        self.drivernode.stop_driver()

    def des_field_changed(self):
        """callback that computes the new currents and publishes them whenever the spinboxes change
        """
        desB=np.zeros(3)
        mag=self.magbox.value()
        azim=self.azimbox.value()/180*pi
        elev=self.elevbox.value()/180*pi
        desB[0]=mag*cos(elev)*cos(azim)
        desB[1]=mag*cos(elev)*sin(azim)
        desB[2]=mag*sin(elev)
        des_currents=self.Bactinv.dot(desB*1e-3)
        self.des_currents_msg.data=des_currents
        #print(des_currents)
        self.des_currents_pub.publish(self.des_currents_msg)

    def getCurrentsString(self,currents):
        s="["
        for i in range(0,self.Ncoils):
            s+="{:+.2f},".format(currents[i])
        s=s[:-1]
        s+="] A"
        return s
    
    def system_state_callback(self,msg):
        """callback for `MDriver` state, reads the currents and updated the currents label

        Args:
            msg (mdriver.msg.status): status of the `MDriver` (see `Status.msg`)
        """
        self.meas_currents_label.setText(self.getCurrentsString(msg.currents_reg))

if __name__ == '__main__':
    # Create the QApplication instance
    app = QApplication(sys.argv)

    import os
    calibration_file=os.getenv("DATABASEPATH")+"/tnb_emns/calibration_data/mininavion_steelcore_steelyoke_14cm/calibration.pkl"

    # Create the main window
    gui = MDriverFieldGUI(calibration_file,6)

    # Show the window
    gui.show()

    # Start the application's event loop
    sys.exit(app.exec_())