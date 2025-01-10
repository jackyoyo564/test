# Random Number Publisher
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64  # Replace with your custom msg if needed
import random

class RandomNumberPublisher(Node):
    def __init__(self):
        super().__init__('random_number_publisher')
        self.publisher_ = self.create_publisher(Int64, 'random_number', 10)
        self.timer = self.create_timer(5.0, self.timer_callback)  # Publish every 5 seconds

    def timer_callback(self):
        msg = Int64()
        msg.data = random.randint(0, 100)  # Generate a random number
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')


def main(args=None):
    rclpy.init(args=args)

    try:
        random_number_publisher = RandomNumberPublisher()
        rclpy.spin(random_number_publisher)
    finally:
        random_number_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()