import rclpy
from rclpy.node import Node
from tutorial_interfaces.srv import Bank
from tsai_package.CLASS import Banks  # 引入 Banks 類別


class BankServer(Node):
    def __init__(self):
        super().__init__('bank_server')
        self.srv = self.create_service(Bank, 'tutorial_interfaces', self.handle_request)
        # 使用字典記錄多個使用者的銀行帳戶
        self.accounts = {}
        self.get_logger().info('Bank Server is ready.')

    def handle_request(self, request, response):
        # 檢查是否已經有此使用者的帳戶
        user_name = request.data  # 使用 data 字段表示用戶名
        if user_name not in self.accounts:
            # 若沒有此帳戶，為新用戶創建一個銀行帳戶
            self.accounts[user_name] = Banks(name=user_name, bankname="XYZ Bank")
            self.get_logger().info(f"Created account for {user_name}.")

        # 取得用戶的帳戶實例
        user_account = self.accounts[user_name]

        if request.cmd == 1:  # 存款
            amount = request.amount  # 直接從 request 取得金額
            result = user_account.save_money(amount)
            response.money = user_account.balance
            self.get_logger().info(result)

        elif request.cmd == 2:  # 提款
            amount = request.amount  # 直接從 request 取得金額
            result = user_account.withdraw_money(amount)
            if "Withdrawal failed" in result:
                response.money = -1  # 表示提款失敗
            else:
                response.money = user_account.balance
            self.get_logger().info(result)

        elif request.cmd == 3:  # 查詢餘額
            response.money = user_account.balance  # 直接返回餘額
            self.get_logger().info(f"Balance for {user_name} is {response.money}.")

        elif request.cmd == 4:  # 關閉
            self.get_logger().info("Shutting down the server.")
            response.money = 0
            self.destroy_node()

        else:  # 無效指令
            response.money = -1
            self.get_logger().warn(f"Invalid command: {request.cmd}")

        return response


def main(args=None):
    rclpy.init(args=args)
    server = BankServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info("Server interrupted by user.")
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
