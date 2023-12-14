#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge
import numpy as np
import cv2

class RobotNode(Node):
    def __init__(self):
        super().__init__("RobotNode")

        self.subscriber = self.create_subscription(Bool, "ArUcoCenterAboveHalf", self.subscription_callback, 10)
        self.prev_message = False # Previous message - change position only on posedge/negedge
        self.jointPublisher = self.create_publisher(JointTrajectory, "scaled_joint_trajectory_controller/joint_trajectory", 10)

    def subscription_callback(self, msg):
        if msg.value != self.prev_message: 
            trajectory = JointTrajectory(
            joint_names=['elbow_joint'],
            points=[
                JointTrajectoryPoint(positions=0, time_from_start=Duration(sec=6, nanosec=0))
                ],
            )
            self.jointPublisher.publish(trajectory)
            self.prev_message = msg.value