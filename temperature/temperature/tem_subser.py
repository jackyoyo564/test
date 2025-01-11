import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from tutorial_interfaces.srv import TemperatureControl


class TemperatureManager(Node):
    def __init__(self):
        super().__init__('temperature_manager')
        self.current_temperature = None
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.temperature_callback,
            10
        )
        self.service = self.create_service(
            TemperatureControl,
            'temperature_control',
            self.handle_temperature_request
        )

    def temperature_callback(self, msg):
        self.current_temperature = msg.data
        self.get_logger().info(f'Received temperature: {self.current_temperature:.2f}°C')

    def handle_temperature_request(self, request, response):
        if self.current_temperature is None:
            response.success = False
            response.message = "No temperature data received yet."
            return response

        # 保存改變前的溫度
        old_temperature = self.current_temperature

        if request.action == "heating":
            self.current_temperature += 10.0
            response.message = f'Temperature increased from {old_temperature:.2f}°C to {self.current_temperature:.2f}°C'
        elif request.action == "cooling":
            self.current_temperature -= 10.0
            response.message = f'Temperature decreased from {old_temperature:.2f}°C to {self.current_temperature:.2f}°C'
        else:
            response.success = False
            response.message = "Invalid action. Use 'heating' or 'cooling'."
            return response

        # 設定回應為成功
        response.success = True

        # 在伺服器端記錄改變前後的溫度
        self.get_logger().info(f'{response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TemperatureManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
