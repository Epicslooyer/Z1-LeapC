import socket
import rclpy
from rclpy.node import Node

class ReceiverNode(Node):
    def __init__(self):
        super().__init__("receiver_node")
        self.pub = self.create_publisher(String, "/gesture", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.bind(("0.0.0.0", 5005)) # Figure out which port for rosserver
        self.get_logger().info("Receiver node initialized")
        self.create_timer(0.001, self.read_packets)
    
    def read_packets(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            arr = [float(x) for x in data.decode().split(',')]
            msg = String()
            msg.data = str(arr)
            self.pub.publish(msg)
        except BlockingIOError:
            return


if __name__ == "__main__":
    rclpy.init()
    node = ReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()