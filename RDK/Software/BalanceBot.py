from Periph.USB2CAN import USB2CAN
from StateMachine import StateMachine
from Communicate import Communicate
from Arm import Arm

USB2CAN_1_PORT = '/dev/ttyUSB0'
USB2CAN_1_BAUD = 2000000


class BalanceBot:
    def __init__(self):
        # 
        self.can = USB2CAN(USB2CAN_1_PORT, USB2CAN_1_BAUD)
        self.comm = Communicate()                               # communication monitor
        self.sm = StateMachine()                                # state machine of robot
        self.arm = Arm()                                        # arm control class

    def recv_can_msg(self):
        """
        process received CAN message from message queue
        :return:
        """
        # received flag

        while self.can.canMsgList:
            msg = self.can.canMsgList.pop()






def modify_number(x):
    x += 10
    return x  # 返回修改后的值

if __name__ == '__main__':
    bot = BalanceBot()
    num = 5
    num = modify_number(num)
    print(num)  # 输出：15
