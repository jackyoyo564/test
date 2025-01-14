import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ReverseCmdVelNode(Node):
    def __init__(self):
        super().__init__('reverse_cmd_vel_node')  # 節點名稱
        self.get_logger().info("ReverseCmdVelNode has been started.")

        # 訂閱默認的 `/turtle1/cmd_vel` topic
        self.subscription = self.create_subscription(
            Twist,
            '/turtle1/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # 新的 topic `/reversed_cmd_vel` 的 publisher
        self.publisher = self.create_publisher(
            Twist,
            '/reversed_cmd_vel',
            10
        )

    def cmd_vel_callback(self, msg):
        # 反轉速度指令
        reversed_msg = Twist()
        reversed_msg.linear.x = -msg.linear.x
        reversed_msg.linear.y = -msg.linear.y
        reversed_msg.linear.z = -msg.linear.z
        reversed_msg.angular.x = -msg.angular.x
        reversed_msg.angular.y = -msg.angular.y
        reversed_msg.angular.z = -msg.angular.z

        # 日誌：輸出接收到的訊息和反轉後的訊息
        self.get_logger().info(
            f"Received: linear={msg.linear.x}, angular={msg.angular.z} | "
            f"Reversed: linear={reversed_msg.linear.x}, angular={reversed_msg.angular.z}"
        )

        # 發布反轉後的指令到新 topic
        self.publisher.publish(reversed_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ReverseCmdVelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
