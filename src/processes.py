import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from multiprocessing import Process

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.i += 1

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscriber = self.create_subscription(
            String,
            'chatter',
            self.callback,
            10
        )
        self.subscriber  # prevent unused variable warning

    def callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def run_talker():
    rclpy.init(args=None)
    talker = Talker()
    rclpy.spin(talker)
    rclpy.shutdown()

def run_listener():
    rclpy.init(args=None)
    listener = Listener()
    rclpy.spin(listener)
    rclpy.shutdown()

if __name__ == '__main__':
    talker_process = Process(target=run_talker)
    listener_process = Process(target=run_listener)

    talker_process.start()
    listener_process.start()

    # In this example, both nodes will run for 10 seconds.
    # Afterward, both processes will be terminated.
    time.sleep(10)

    talker_process.terminate()
    listener_process.terminate()

    talker_process.join()
    listener_process.join()
