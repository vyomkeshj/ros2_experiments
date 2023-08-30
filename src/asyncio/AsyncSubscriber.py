import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import asyncio


class AsyncSubscriber(Node):

    def __init__(self):
        super().__init__('async_subscriber')
        self.subscription = self.create_subscription(
            Int32,
            'tropicana',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        asyncio.create_task(self.async_callback(msg))

    async def spin(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            await asyncio.sleep(0.1)


def main(args=None):
    rclpy.init(args=args)

    async_subscriber = AsyncSubscriber()

    loop = asyncio.get_event_loop()
    loop.run_until_complete(async_subscriber.spin())
    loop.close()

    async_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
