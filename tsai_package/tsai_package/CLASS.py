class Banks:
    def __init__(self, name, bankname):
        """
        Initialize the bank account
        :param name: User's name
        :param bankname: Bank name
        """
        self.name = name
        self.bankname = bankname
        self.balance = 0  # Default balance is 0

    def save_money(self, amount):
        """
        Deposit method
        :param amount: Amount to deposit
        """
        if amount > 0:
            self.balance += amount
            return f"Deposit successful, current balance: {self.balance}"
        else:
            return "Deposit amount must be greater than 0!"

    def withdraw_money(self, amount):
        """
        Withdrawal method
        :param amount: Amount to withdraw
        """
        if amount <= 0:
            return "Withdrawal amount must be greater than 0!"
        if amount > self.balance:
            return "Insufficient balance, withdrawal failed!"
        self.balance -= amount
        return f"Withdrawal successful, current balance: {self.balance}"

    def get_balance(self):
        """
        Check balance method
        :return: Current account balance
        """
        return f"User {self.name}, Bank {self.bankname}, Current balance: {self.balance}"
