import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random


class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher_ = self.create_publisher(Float32, 'temperature', 10)
        self.timer = self.create_timer(1.0, self.publish_temperature)  # 每秒發佈一次
        self.get_logger().info('Temperature Publisher started.')

    def publish_temperature(self):
        temp = random.uniform(-50, 50)  # 隨機生成 -50 到 50 之間的溫度
        msg = Float32()
        msg.data = temp
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {temp:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = TemperaturePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Temperature Publisher stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
