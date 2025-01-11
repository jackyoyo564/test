import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from point_interfaces.srv import CalculateDistance
import math


class DistanceService(Node):
    def __init__(self):
        super().__init__('distance_service')
        self.srv = self.create_service(CalculateDistance, 'calculate_distance', self.calculate_distance_callback)
        self.get_logger().info('Distance Service ready.')

    def calculate_distance_callback(self, request, response):
        # 取得兩個點的座標
        x1, y1, z1 = request.point1.x, request.point1.y, request.point1.z
        x2, y2, z2 = request.point2.x, request.point2.y, request.point2.z

        # 計算歐幾里得距離
        response.distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)

        # 日誌輸出
        self.get_logger().info(f'Received points: ({x1}, {y1}, {z1}) and ({x2}, {y2}, {z2})')
        self.get_logger().info(f'Calculated distance: {response.distance:.2f}')

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DistanceService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Distance Service...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
