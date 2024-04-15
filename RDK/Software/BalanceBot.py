from Periph.USB2CAN import USB2CAN

USB2CAN_1_PORT = '/dev/ttyUSB0'
USB2CAN_1_BAUD = 1000000
USB2CAN_2_PORT = '/dev/ttyUSB1'
USB2CAN_2_BAUD = 1000000


class BalanceBot:
    def __init__(self):
        # 

        # 硬件外设
        self.can = [USB2CAN(USB2CAN_1_PORT, USB2CAN_1_BAUD),
                    USB2CAN(USB2CAN_2_PORT, USB2CAN_2_BAUD)]

        # self.imu =
        
    # def recv_can_msg(self):


if __name__ == '__main__':
    bot = BalanceBot()

