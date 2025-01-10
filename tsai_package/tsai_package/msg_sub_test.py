# Random Number Subscriber
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64  # Replace with your custom msg if needed
import random

class RandomNumberSubscriber(Node):
    def __init__(self):
        super().__init__('random_number_subscriber')
        self.subscription = self.create_subscription(
            Int64,
            'random_number',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    try:
        random_number_subscriber = RandomNumberSubscriber()
        rclpy.spin(random_number_subscriber)
    finally:
        random_number_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
