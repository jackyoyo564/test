import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import Bank


class BankClient(Node):
    def __init__(self):
        super().__init__('bank_client')
        self.client = self.create_client(Bank, 'tutorial_interfaces')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info('Waiting for the bank service...')
        self.request = Bank.Request()

    def send_request(self):
        # Request user name first
        self.request.data = input("Enter your username: ")

        # Display menu
        print("\nPlease choose an operation:")
        print("1. Deposit")
        print("2. Withdraw")
        print("3. Check balance")
        print("4. Exit")
        
        try:
            self.request.cmd = int(input("Enter your command (1-4): "))
        except ValueError:
            print("Invalid input. Please enter a number between 1 and 4!")
            return

        if self.request.cmd not in [1, 2, 3, 4]:
            print("Invalid command. Please try again!")
            return

        # If the command is Deposit or Withdraw, ask for the amount
        if self.request.cmd in [1, 2]:
            try:
                self.request.amount = int(input("Enter the amount: "))
            except ValueError:
                print("Invalid amount. Please enter a valid number.")
                return

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            if self.request.cmd == 3:  # If the command is to check balance
                print(f"User {self.request.data}'s current balance is: {future.result().money}")
            else:
                print(f"Operation successful for user {self.request.data}, returned amount: {future.result().money}")
        else:
            print("Service response failed!")


def main(args=None):
    rclpy.init(args=args)
    client = BankClient()

    while rclpy.ok():
        client.send_request()
        if client.request.cmd == 4:  # Exit command
            break

    client.get_logger().info("Client shutting down.")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
