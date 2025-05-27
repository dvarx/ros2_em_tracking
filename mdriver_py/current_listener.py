#!/usr/bin/env python3

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from mdriver.srv import StateTransition
from mdriver.msg import Status
import time
import rclpy
import matplotlib.pyplot as plt

currents=[]
times=[]

class CurrentListenerNode(Node):
    def __init__(self):
        super().__init__("current_listener")

        self.get_logger().info("starting current_listener_node")

        self.no_currents=512
        self.counter=0

        #define listener for system state
        self.subscription=self.create_subscription(
            Status,
            '/mdriver/system_state',
            self.state_callback,
            1
        )

    def state_callback(self,msg):
        if(self.counter>self.no_currents):
            self.destroy_subscription(self.subscription)
            self.plot_currents()

        currents.append(msg.currents_reg)
        times.append(self.get_clock().now())
        self.counter+=1
        self.get_logger().info("counter %d"%self.counter)

    def plot_currents(self):
        times_s=[(times[i]-times[0]).nanoseconds*1e-9 for i in range(1,len(times))]
        times_s=[0]+times_s
        plt.plot(times_s,currents)
        plt.xlabel("Time [s]")
        plt.ylabel("Current [A]")
        plt.legend(["Channel %d"%i for i in range(1,6+1)])
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = CurrentListenerNode()

    rclpy.spin(node)







if __name__ == "__main__":
    main()