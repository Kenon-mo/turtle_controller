import rclpy
from turtle_controller.camera_node import CameraNode

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__  == '__main__':
    main()