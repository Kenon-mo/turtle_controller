#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__("CameraNode")

        self.declareAllParams()
        self.getAllParams()

        self.camera = cv2.VideoCapture(self.cameraDevice.value)
        SPF = 1.0 / float(self.cameraFPS.value) # 'seconds per frame'
        self.centerPointPublisher = self.create_publisher(Bool, "ArUcoCenterAboveHalf", 10)

        self.arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
        self.arucoParams = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.arucoDict, self.arucoParams)

        if bool(self.showPreview.value):
            cv2.namedWindow("Preview")
            self.create_timer(SPF, self.showFrame)
        else:
            self.create_timer(SPF, self.timer_callback)

    def declareAllParams(self):
        self.declare_parameter("CameraFPS", "30")
        self.declare_parameter("ShowPreview", "true")
        self.declare_parameter("CameraDevice", "/dev/video0")
        self.declare_parameter("ArUcoID", "0")

    def getAllParams(self):
        self.cameraFPS = self.get_parameter("CameraFPS")
        self.showPreview = self.get_parameter("ShowPreview")
        self.cameraDevice = self.get_parameter("CameraDevice")
        self.arucoID = self.get_parameter("ArUcoID")

    def timer_callback(self):
        frame = self.getFrame()
        corners = self.detectArUcoCorners(frame)
        y, x, canals = frame.shape
        if corners != None:
            center = self.getArUcoCenter(corners)
            mes = Bool()
            if center.y < y/2:
                mes.data = True
            else:
                mes.data = False
            self.centerPointPublisher.publish(mes)

    def getFrame(self):
        if self.camera.isOpened():
            ret, frame = self.camera.read()
            if ret:
                cv2.waitKey(1) # Wait key needed for output
                return frame
            else:
                self.get_logger().info("Failed to get image from camera")
                return None

    def detectArUcoCorners(self, frame):
        (corners, ids, rejected) = self.detector.detectMarkers(frame)
        if len(corners) > 0:
            for id in ids:
                if id[0] == int(self.arucoID.value):
                    return corners
            self.get_logger().info("ID: " + str(self.arucoID.value) + " not found!")
        return None

    def getArUcoCenter(self, corners):
        center = Point()
        for x, y in corners[0][0]:
            center.x += float(x)
            center.y += float(y)
        center.x /= 4
        center.y /= 4
        center.z = float(0.0)
        return center

    def showFrame(self):
        frame = self.getFrame()
        corners = self.detectArUcoCorners(frame)
        y, x, canals = frame.shape
        frame[int(y/2), 0:x] = (0, 255, 0)
        if corners != None:
            center = self.getArUcoCenter(corners)
            frame[int(center.y)-5:int(center.y)+5, int(center.x)-5:int(center.x)+5] = (0, 0, 255)
            mes = Bool()
            if center.y < y/2:
                mes.data = True
            else:
                mes.data = False
            self.centerPointPublisher.publish(mes)
        cv2.imshow("Preview", frame)

    def __del__(self):
        cv2.destroyAllWindows()
        self.camera.release()


