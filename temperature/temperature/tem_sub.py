import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32


class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )
        self.subscription  # 防止垃圾回收
        self.get_logger().info('Temperature Subscriber started.')

    def temperature_callback(self, msg):
        temp = msg.data
        self.get_logger().info(f'Received: {temp:.2f}')
        if temp < -30:
            self.get_logger().warn(f'Warning: Temperature too low! ({temp:.2f}°C)')
        elif temp > 30:
            self.get_logger().warn(f'Warning: Temperature too high! ({temp:.2f}°C)')


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Temperature Subscriber stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
