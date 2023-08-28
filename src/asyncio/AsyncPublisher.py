import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import asyncio


class AsyncPublisher(Node):

    def __init__(self):
        super().__init__('async_publisher')
        self.publisher_ = self.create_publisher(Int32, 'tropicana', 10)
        self.count = 0

    async def publish_message(self):
        msg = Int32()
        while True:
            msg.data = self.count
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%d"' % msg.data)
            self.count += 1
            await asyncio.sleep(1)


def main(args=None):
    rclpy.init(args=args)

    async_publisher = AsyncPublisher()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(async_publisher.publish_message())
    loop.close()

    async_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
