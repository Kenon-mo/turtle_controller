#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2

class CameraNode(Node):
    def __init__(self):
        super().__init__("CameraNode")

        self.declareAllParams()
        self.getAllParams()

        self.camera = cv2.VideoCapture(self.cameraDevice.value)

        if bool(self.showPreview.value):
            cv2.namedWindow("Preview")
            self.create_timer(1.0 / float(self.cameraFPS.value), self.showFrame)

        


    def declareAllParams(self):
        self.declare_parameter("CameraFPS", "30")
        self.declare_parameter("ShowPreview", "true")
        self.declare_parameter("CameraDevice", "/dev/video0")

    def getAllParams(self):
        self.cameraFPS = self.get_parameter("CameraFPS")
        self.showPreview = self.get_parameter("ShowPreview")
        self.cameraDevice = self.get_parameter("CameraDevice")

    def timer_callback(self):
        self.showFrame()

    def showFrame(self):
        if self.camera.isOpened(): # try to get the first frame
            ret, frame = self.camera.read()
            if ret:
                cv2.imshow("Preview", frame)
                self.get_logger().info("Image should be visible, image sum = " + str(frame.sum()))
                cv2.waitKey(1) # Wait key needed for output
                
            else:
                self.get_logger().info("Failed to open self.camera: /dev/video0")
        else:
            self.get_logger().info("Failed to open self.camera: /dev/video0")

    def __del__(self):
        cv2.destroyAllWindows()
        self.camera.release()

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__  == '__main__':
    main()
