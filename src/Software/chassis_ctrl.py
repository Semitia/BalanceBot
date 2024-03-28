from ..Periph import USB2CAN
from ..Software import KneeMotor, WheelMotor


# number of motors
LEFT = 0
RIGHT = 1
FRONT = 0
BACK = 1
# CAN ID
LEFT_FRONT_CAN_ID = 0x101
LEFT_BACK_CAN_ID = 0x102
RIGHT_FRONT_CAN_ID = 0x103
RIGHT_BACK_CAN_ID = 0x104
LEFT_WHEEL_CAN_ID = 0x105
RIGHT_WHEEL_CAN_ID = 0x106
MOTOR_SEND_CAN1_ID = 0x100
MOTOR_SEND_CAN2_ID = 0x200
ARM_SEND1_CAN_ID = 0x301
ARM_SEND2_CAN_ID = 0x302
TOF_CAN_ID = 0x310
IMU_CAN_ID = 0x320


class ChassisCtrl:
    """
    机器人底盘控制类
    """
    def __init__(self, can1, can2):
        self.can = [can1, can2]
        self.knee = [[KneeMotor(LEFT_FRONT_CAN_ID), KneeMotor(LEFT_BACK_CAN_ID)],
                     [KneeMotor(RIGHT_FRONT_CAN_ID), KneeMotor(RIGHT_BACK_CAN_ID)] ]
        self.wheel = [WheelMotor(LEFT_WHEEL_CAN_ID), WheelMotor(RIGHT_WHEEL_CAN_ID)]

