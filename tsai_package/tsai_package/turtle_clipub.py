import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist  # To send movement commands to the turtle
from tutorial_interfaces.srv import AddThreeInts
import random

class ClientPublisher(Node):

    def __init__(self):
        super().__init__('client_publisher')
        self.client = self.create_client(AddThreeInts, 'add_three_ints')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, retrying...')
        
        # Set up publisher to send movement commands to the turtle (cmd_vel)
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        # Timer to periodically ask for user input
        self.timer = self.create_timer(1.0, self.timer_callback)  # Run every second
    
    def timer_callback(self):
        
        
        # Send a random movement to the turtle
        self.send_random_movement()

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f'Received response from server: {response.sum}')
        except Exception as e:
            self.get_logger().error(f'Error calling service: {e}')
    
    def send_random_movement(self):
        # Randomly generate linear and angular velocities for the turtle
        linear_velocity = random.uniform(-2.0, 2.0)  # Random speed between -2 and 2
        angular_velocity = random.uniform(-2.0, 2.0)  # Random angular speed between -2 and 2
        
        # Create a Twist message to control the turtle
        twist = Twist()
        twist.linear.x = linear_velocity
        twist.angular.z = angular_velocity
        
        # Publish the Twist message to the cmd_vel topic to move the turtle
        self.publisher.publish(twist)
        self.get_logger().info(f'Sent random movement: linear.x={linear_velocity}, angular.z={angular_velocity}')

    def send_request(self):
        # Prompt user to enter three numbers
        self.get_logger().info("Please enter three integer numbers (press Enter after each one):")
        
        try:
            num1 = int(input("Enter the first number: "))
            num2 = int(input("Enter the second number: "))
            num3 = int(input("Enter the third number: "))
        except ValueError:
            self.get_logger().error("Invalid input! Please make sure to enter integers.")
            return
        
        # Call the service to add the integers
        request = AddThreeInts.Request()
        request.a = num1
        request.b = num2
        request.c = num3
        self.future = self.client.call_async(request)
        
        # Handle the service response
        self.future.add_done_callback(self.response_callback)



def main(args=None):
    rclpy.init(args=args)
    client_publisher = ClientPublisher()

    client_publisher.send_request()

    rclpy.spin(client_publisher)
    client_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
