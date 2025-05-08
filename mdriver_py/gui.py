#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QLabel, QVBoxLayout, QDoubleSpinBox
from PyQt5.QtCore import Qt, QThread
from mdriver_node import MDriverNode
from std_msgs.msg import Float32MultiArray
from mdriver.msg import Status
import rclpy
import numpy as np

class ROS2Thread(QThread):
    def __init__(self, node):
        super().__init__()
        self._node = node
        self._running = True

    def run(self):
        rclpy.spin(self._node)
        print("ROS 2 spin ended.")

    def stop(self):
        self._running = False
        self._node.destroy_node()
        rclpy.shutdown()
        self.wait()

class SimpleGUI(QWidget):
    def __init__(self,Ncoils):
        self.Ncoils=Ncoils

        rclpy.init()
        self.drivernode=MDriverNode()

        self.des_currents_pub=self.drivernode.create_publisher(
            Float32MultiArray,
            "/mdriver/des_currents_reg",
            10
        )
        self.des_currents_msg=Float32MultiArray()
        self.des_currents_msg.data=[0.0]*Ncoils


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

        # Create two text labels
        self.label1 = QLabel('Measured Currents')
        self.label1.setAlignment(Qt.AlignCenter) # Center align the text

        self.meas_currents_label = QLabel('This is the second text label.')
        self.meas_currents_label.setAlignment(Qt.AlignCenter) # Center align the text
        self.meas_currents_label.setStyleSheet("font-family: Monospace;")

        # Create three clickable buttons
        self.enable_button = QPushButton('Enable')
        self.run_regular_button = QPushButton('Run Regular')
        self.stop_button = QPushButton('Stop')

        # Spinboxes for currents
        self.sboxs=[]
        for i in range(0,Ncoils):
            sbox=QDoubleSpinBox()
            self.sboxs.append(sbox)
            sbox.setMinimum(-5)
            sbox.setMaximum(5)
            sbox.setSingleStep(0.1)
            sbox.valueChanged.connect(self.des_currents_changed)


        # Connect buttons to simple print functions (can be customized)
        self.enable_button.clicked.connect(self.enable_clicked)
        self.run_regular_button.clicked.connect(self.run_regular_clicked)
        self.stop_button.clicked.connect(self.stop_clicked)

        # Add widgets to the layout in vertical order
        layout.addWidget(self.label1)
        layout.addWidget(self.meas_currents_label)
        layout.addWidget(self.enable_button)
        layout.addWidget(self.run_regular_button)
        layout.addWidget(self.stop_button)
        for i in range(0,Ncoils):
            layout.addWidget(self.sboxs[i])

        # Set the layout for the main window
        self.setLayout(layout)

        self.ros2_thread=ROS2Thread(self.drivernode)
        self.ros2_thread.start()


    def enable_clicked(self):
        self.drivernode.enable_driver()

    def run_regular_clicked(self):
        self.drivernode.run_regular_driver()

    def stop_clicked(self):
        self.drivernode.stop_driver()

    def des_currents_changed(self):
        for i in range(0,6):
            self.des_currents_msg.data[i]=self.sboxs[i].value()
        self.des_currents_pub.publish(self.des_currents_msg)

    def getCurrentsString(self,nums):
        s="["
        for i in range(0,self.Ncoils):
            s+="{:+.2f},".format(nums[i])
        s=s[:-1]
        s+="]"
        return s
    
    def system_state_callback(self,msg):
        self.meas_currents_label.setText(self.getCurrentsString(msg.currents_reg))


if __name__ == '__main__':
    # Create the QApplication instance
    app = QApplication(sys.argv)


    # Create the main window
    gui = SimpleGUI(Ncoils=6)

    # Show the window
    gui.show()

    # Start the application's event loop
    sys.exit(app.exec_())