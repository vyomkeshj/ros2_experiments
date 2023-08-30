from typing import TypedDict

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import random
import json


class NewDatapoint(TypedDict):
    value: int
    label: str


class AsyncPublisher(Node):

    def __init__(self):
        super().__init__('async_publisher')
        self.publisher_ = self.create_publisher(String, 'tropicana', 10)
        self.count = 0

    async def publish_message(self):
        msg = String()
        while True:
            data_point = NewDatapoint(value=random.randint(1, 10), label="1")
            msg.data = json.dumps(data_point)
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
            self.count += 3
            await asyncio.sleep(4)


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
