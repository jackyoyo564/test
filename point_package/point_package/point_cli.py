import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from point_interfaces.srv import CalculateDistance


class DistanceClient(Node):
    def __init__(self):
        super().__init__('distance_client')
        self.cli = self.create_client(CalculateDistance, 'calculate_distance')

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service is now available.')

    def send_request(self, point1, point2):
        # 建立請求物件並填入座標
        request = CalculateDistance.Request()
        request.point1 = point1
        request.point2 = point2

        # 發送請求並等待結果
        future = self.cli.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info(f'Received distance: {future.result().distance:.2f}')
        else:
            self.get_logger().error('Failed to call service!')


def main(args=None):
    rclpy.init(args=args)

    # 讓使用者輸入座標
    print("Enter coordinates for point 1 (x, y, z):")
    x1 = float(input("x1: "))
    y1 = float(input("y1: "))
    z1 = float(input("z1: "))

    print("Enter coordinates for point 2 (x, y, z):")
    x2 = float(input("x2: "))
    y2 = float(input("y2: "))
    z2 = float(input("z2: "))

    # 建立 Point 物件
    point1 = Point(x=x1, y=y1, z=z1)
    point2 = Point(x=x2, y=y2, z=z2)

    # 啟動客戶端並發送請求
    client = DistanceClient()
    client.send_request(point1, point2)

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
