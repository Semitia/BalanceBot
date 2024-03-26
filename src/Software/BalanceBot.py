

USB2CAN_1_PORT = '/dev/ttyUSB0'
USB2CAN_1_BAUD = 1000000
USB2CAN_2_PORT = '/dev/ttyUSB1'
USB2CAN_2_BAUD = 1000000

# number of motors
LEFT = 0
RIGHT = 1
FRONT = 0
BACK = 1
# CAN id dict
# CAN1_ID_DICT = {
#     'MOTOR_SEND':  0x100,
#     'LEFT_FRONT':  0x101,
#     'LEFT_BACK':   0x102,
#     'RIGHT_FRONT': 0x103,
#     'RIGHT_BACK':  0x104
# }
# CAN2_ID_DICT = {
#     'MOTOR_SEND':  0x200,
#     'ARM_SEND1':   0x301,
#     'ARM_SEND2':   0x302,
#     'LEFT_WHEEL':  0x105,
#     'RIGHT_WHEEL': 0x106,
#     'TOF':         0x310,
#     'IMU':         0x320,
# }
LEFT_FRONT_CAN_ID  = 0x101
LEFT_BACK_CAN_ID   = 0x102
RIGHT_FRONT_CAN_ID = 0x103
RIGHT_BACK_CAN_ID  = 0x104
LEFT_WHEEL_CAN_ID  = 0x105
RIGHT_WHEEL_CAN_ID = 0x106
MOTOR_SEND_CAN1_ID = 0x100
MOTOR_SEND_CAN2_ID = 0x200
ARM_SEND1_CAN_ID   = 0x301
ARM_SEND2_CAN_ID   = 0x302
TOF_CAN_ID         = 0x310
IMU_CAN_ID         = 0x320

class BalanceBot:
    def __init__(self):
        # 
        
        # 外设
        self.can = [USB2CAN(USB2CAN_1_PORT, USB2CAN_1_BAUD),
                    USB2CAN(USB2CAN_2_PORT, USB2CAN_2_BAUD)]
        self.knee = [   [Motor(LEFT_FRONT_CAN_ID), Motor(LEFT_BACK_CAN_ID)],
                        [Motor(RIGHT_FRONT_CAN_ID), Motor(RIGHT_BACK_CAN_ID)] ]
        self.wheel = [Motor(LEFT_WHEEL_CAN_ID), Motor(RIGHT_WHEEL_CAN_ID)]
        
        
    # def recv_can_msg(self):
        
