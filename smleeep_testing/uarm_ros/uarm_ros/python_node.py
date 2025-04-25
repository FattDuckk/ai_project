import rclpy

from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    node1 = Node("python_node")
    node1.get_logger().info("Hello world")

    rclpy.spin(node1)

    rclpy.shutdown()

if __name__ == "__main__":
    main()
