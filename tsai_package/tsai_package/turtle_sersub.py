import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute
from tutorial_interfaces.srv import AddThreeInts  # Use AddThreeInts service

class ServerSubscriber(Node):

    def __init__(self):
        super().__init__('server_subscriber')

        # Set up service to handle addition requests
        self.srv = self.create_service(AddThreeInts, 'add_three_ints', self.add_three_ints_callback)
        
        # Set up subscriber to receive random position from the publisher
        self.subscription = self.create_subscription(String, 'random_position', self.position_callback, 10)
        self.subscription  # Keep reference to prevent garbage collection
        
        # Set up client to call teleport service to move the turtle
        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Teleport service not available, retrying...')

    def add_three_ints_callback(self, request, response):
        # Handle the addition service
        self.get_logger().info(f'Received numbers: {request.a}, {request.b}, {request.c}')
        response.sum = request.a + request.b + request.c  # Add the three numbers
        self.get_logger().info(f'Returning sum: {response.sum}')
        return response

    def position_callback(self, msg):
        # Display received random position
        self.get_logger().info(f'Received random position: {msg.data}')
        
        # Parse the random position to extract x, y coordinates
        position_data = msg.data.strip('Position: ()')  # Remove "Position: (" and ")"
        x, y = map(int, position_data.split(','))  # Convert to integer values
        
        # Call teleport service to move the turtle to the new position
        self.teleport(x, y)

    def teleport(self, x, y):
        # Create a request to teleport the turtle to a new position
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = 0.0  # Keep the orientation the same
        self.teleport_client.call_async(request)
        self.get_logger().info(f'Teleporting turtle to position: ({x}, {y})')

def main(args=None):
    rclpy.init(args=args)
    server_subscriber = ServerSubscriber()
    rclpy.spin(server_subscriber)
    server_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
