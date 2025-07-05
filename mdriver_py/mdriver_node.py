#!/usr/bin/env python3

"""
This script demonstrates the communication with the `MDriver` and defines a python wrapper class for communication with the `MDriver`
"""

from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from mdriver.srv import StateTransition
import time
import rclpy

class MDriverNode(Node):
    def __init__(self):
        super().__init__("mdriver_test_py")

        self.get_logger().info("starting driver test node")

        # Publisher for desired currents
        self.currents_pub = self.create_publisher(
            Float32MultiArray,
            "/mdriver/des_currents_reg",
            10
        )

        # Publisher for desired duty cycles
        self.duties_pub = self.create_publisher(
            Float32MultiArray,
            "/mdriver/des_duties",
            10
        )

        # Service client for ENABLE
        self.enable_cli = self.create_client(
            StateTransition,
            "/mdriver/enable"
        )
        while not self.enable_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for '/mdriver/enable' service...")

        # Service client for RUN_REGULAR
        self.run_regular_cli = self.create_client(
            StateTransition,
            "/mdriver/run_regular"
        )
        while not self.run_regular_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for '/mdriver/run_regular' service...")

        # Service client for STOP
        self.stop_cli = self.create_client(
            StateTransition,
            "/mdriver/stop"
        )
        while not self.stop_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for '/mdriver/stop' service...")

    def enable_driver(self):
        request = StateTransition.Request()
        request.enable = [True] * 6
        future = self.enable_cli.call_async(request)
        self.get_logger().info("Sending 'ENABLE' request...")
        rclpy.spin_until_future_complete(self, future)

    def run_regular_driver(self):
        request = StateTransition.Request()
        request.enable = [True] * 6
        future = self.run_regular_cli.call_async(request)
        self.get_logger().info("Sending 'RUN_REGULAR' request...")
        rclpy.spin_until_future_complete(self, future)

    def stop_driver(self):
        request = StateTransition.Request()
        request.enable = [True] * 6
        future = self.stop_cli.call_async(request)
        self.get_logger().info("Sending 'STOP' request...")
        rclpy.spin_until_future_complete(self, future)

    def set_currents(self,currents):
        des_currents_msg = Float32MultiArray()
        des_currents_msg.data = list(currents)
        self.currents_pub.publish(des_currents_msg)

    def control_magnets(self):
        # Message for the currently desired current vector
        des_currents_msg = Float32MultiArray()
        des_currents_msg.data = [0.0] * 6

        # Set all duties to 50%
        self.get_logger().info("Setting all duties to 50%")
        des_duties_msg = Float32MultiArray()
        des_duties_msg.data = [0.5] * 6
        self.duties_pub.publish(des_duties_msg)
        self.get_logger().info("Published desired duties.")
        self.create_timer(1.0, lambda: None)
        rclpy.spin_once(self, timeout_sec=1.0)

        testcurrent = 1.0
        test_currents = [0.0] * 6
        # Apply current in each coil separately
        for i in range(6):
            test_currents = [0.0] * 6
            test_currents[i] = testcurrent
            self.get_logger().info(f"Applying current vector:\n{test_currents}")
            des_currents_msg.data = test_currents
            self.currents_pub.publish(des_currents_msg)
            self.get_logger().info("Published desired currents.")
            time.sleep(10)

        # Apply zero currents
        self.get_logger().info("Applying zero currents\n")
        test_currents = [0.0] * 6
        des_currents_msg.data = test_currents
        self.currents_pub.publish(des_currents_msg)
        self.get_logger().info("Published desired currents.")
        time.sleep(10)

def main(args=None):
    rclpy.init(args=args)
    node = MDriverNode()

    node.enable_driver()
    time.sleep(1)
    node.run_regular_driver()
    time.sleep(1)
    node.control_magnets()
    time.sleep(1)
    node.stop_driver()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()