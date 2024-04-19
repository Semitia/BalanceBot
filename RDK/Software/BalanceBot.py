from Periph.USB2CAN import USB2CAN

USB2CAN_1_PORT = '/dev/ttyUSB0'
USB2CAN_1_BAUD = 2000000


class BalanceBot:
    def __init__(self):
        # 

        # 硬件外设
        self.can = USB2CAN(USB2CAN_1_PORT, USB2CAN_1_BAUD)

        # state machine of robot
        
    # def recv_can_msg(self):


if __name__ == '__main__':
    bot = BalanceBot()

