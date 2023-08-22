import rclpy
from rclpy.node import Node

class Node1(Node):
    def __init__(self):
        super().__init__('node_1')
        self.get_logger().info('Node 1 is up!')

class Node2(Node):
    def __init__(self):
        super().__init__('node_2')
        self.get_logger().info('Node 2 is up!')

class Node3(Node):
    def __init__(self):
        super().__init__('node_3')
        self.get_logger().info('Node 3 is up!')

def main():
    rclpy.init()

    node1 = Node1()
    node2 = Node2()
    node3 = Node3()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node1)
    executor.add_node(node2)
    executor.add_node(node3)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node1.destroy_node()
        node2.destroy_node()
        node3.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
