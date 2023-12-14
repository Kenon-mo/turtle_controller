import rclpy
from ur3e_aruco_controller.robot_node import RobotNode

def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__  == '__main__':
    main()