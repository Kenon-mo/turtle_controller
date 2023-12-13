#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2

class RobotNode(Node):
    def __init__(self):
        super().__init__("RobotNode")

        self.subscriber = self.create_subscription(Bool, "ArUcoCenterAboveHalf", self.subscription_callback, 10)
        self.prev_message = False # Previous message - change position only on posedge/negedge


    def subscription_callback(self):
        pass