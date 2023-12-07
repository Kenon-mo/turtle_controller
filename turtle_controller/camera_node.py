#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__("CameraNode")

        self.declareAllParams()
        self.getAllParams()

        self.camera = cv2.VideoCapture(self.cameraDevice.value)
        SPF = 1.0 / float(self.cameraFPS.value) # 'seconds per frame'
        self.create_publisher(Image, "CameraNodeImage", 10)

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

    def getAllParams(self):
        self.cameraFPS = self.get_parameter("CameraFPS")
        self.showPreview = self.get_parameter("ShowPreview")
        self.cameraDevice = self.get_parameter("CameraDevice")

    def timer_callback(self):
        frame = self.getFrame()
        corners = self.detectArUcoCorners(frame)


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
        return corners

    def getAvgCornerY(self, corners):
        avgy = 0
        for corner in corners:
            avgy += corner.y
        avgy /= len(corners)
        return avgy

    def showFrame(self):
        frame = self.getFrame()
        cv2.imshow("Preview", frame)
        self.get_logger().info("Image should be visible, image sum = " + str(frame.sum()))
        corners = self.detectArUcoCorners(frame)
        for corner in corners:
            self.get_logger().info("ArUco visible at: " + str(corner[0][0][1]))


    def __del__(self):
        cv2.destroyAllWindows()
        self.camera.release()


