import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import TemperatureControl


class TemperatureClient(Node):
    def __init__(self):
        super().__init__('temperature_client')
        self.cli = self.create_client(TemperatureControl, 'temperature_control')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, action):
        request = TemperatureControl.Request()
        request.action = action

        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(response.message)
            else:
                self.get_logger().error(response.message)
        else:
            self.get_logger().error('Failed to call service!')


def main(args=None):
    rclpy.init(args=args)

    client = TemperatureClient()

    while True:
        action = input("Enter action ('heating' or 'cooling', or 'exit' to quit): ").strip()
        if action == 'exit':
            break
        client.send_request(action)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
