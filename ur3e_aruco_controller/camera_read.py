import rclpy
from ur3e_aruco_controller.camera_node import CameraNode

def main(args=None):
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__  == '__main__':
    main()