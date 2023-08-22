import rclpy
from rclpy.node import Node

def list_nodes():
    rclpy.init()
    node = Node('list_nodes_node')

    # Get the list of nodes
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    for name, namespace in node_names_and_namespaces:
        full_name = f"{namespace}/{name}" if namespace else name
        print(full_name)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    list_nodes()
