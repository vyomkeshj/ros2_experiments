import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from multiprocessing import Process, Event

class Talker(Node):
    def __init__(self, event):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0
        self.shutdown_event = event

    def timer_callback(self):
        if self.shutdown_event.is_set():
            rclpy.shutdown()
            return
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.i += 1

class Listener(Node):
    def __init__(self, event):
        super().__init__('listener')
        self.subscriber = self.create_subscription(String, 'chatter', self.callback, 10)
        self.shutdown_event = event

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        if self.shutdown_event.is_set():
            rclpy.shutdown()

def run_talker(event):
    rclpy.init(args=None)
    talker = Talker(event)
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

def run_listener(event):
    rclpy.init(args=None)
    listener = Listener(event)
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    shutdown_event = Event()

    talker_process = Process(target=run_talker, args=(shutdown_event,))
    listener_process = Process(target=run_listener, args=(shutdown_event,))

    talker_process.start()
    listener_process.start()

    # Let nodes run for 10 seconds.
    time.sleep(10)

    # Set the event, signaling the nodes to shut down.
    shutdown_event.set()

    # Now, we just wait for both processes to finish gracefully.
    talker_process.join()
    listener_process.join()
