#!/usr/bin/env python3

# ROS2 Packages
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Int64
from test_msgs.srv import BasicTypes

# Basestation Node Package
import os, sys
from ros import mainNode

def main():
    # Initialize ROS 2 system
    rclpy.init()

    # Create an instance of the mainNode class
    node = mainNode()

    # Perform operations with the node
    rclpy.spin(node)

    # Destroy the node and release resources
    node.destroy_node()
    rclpy.shutdown()

main()