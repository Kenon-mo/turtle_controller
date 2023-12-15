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
        self.subscriber = self.create_subscription(Bool, "/ArUcoCenterAboveHalf", self.subscription_callback, 10)
        self.subscriber # prevent unused variable warning
        self.prev_message = False # Previous message - change position only on posedge/negedge
        self.jointPublisher = self.create_publisher(JointTrajectory, "/scaled_joint_trajectory_controller/joint_trajectory", 10)
        self.get_logger().info("RobotNode created!")

    def subscription_callback(self, msg):
        if msg.data != self.prev_message:
            if msg.data:
                trajectory = JointTrajectory( # moving forward if msg is true
                    joint_names=[
                        "elbow_joint",
                        "shoulder_lift_joint",
                        "shoulder_pan_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint",
                    ],
                    points=[
                        JointTrajectoryPoint(positions=[0.5235988, -0.5235988, 0, -1.570796, -1.570796, 0], time_from_start=Duration(sec=6, nanosec=0))
                        ],
                )
            else:
                trajectory = JointTrajectory( # moving backward if msg is false
                    joint_names=[
                        "elbow_joint",
                        "shoulder_lift_joint",
                        "shoulder_pan_joint",
                        "wrist_1_joint",
                        "wrist_2_joint",
                        "wrist_3_joint",
                    ],
                    points=[
                        JointTrajectoryPoint(positions=[1.570796, -3.141593, 0, -1.570796, -1.570796, 0], time_from_start=Duration(sec=6, nanosec=0))
                        ],
                )
            self.jointPublisher.publish(trajectory)
            self.prev_message = msg.data