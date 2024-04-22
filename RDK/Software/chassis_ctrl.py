from Software import KneeMotor, WheelMotor
from LegCtrl import LegCtrl
from Algorithm import LQR

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

# dictionary: msg id -> motor number
msg2motor = {
    LEFT_FRONT_CAN_ID: 0,
    LEFT_BACK_CAN_ID: 1,
    RIGHT_FRONT_CAN_ID: 2,
    RIGHT_BACK_CAN_ID: 3,
    LEFT_WHEEL_CAN_ID: 4,
    RIGHT_WHEEL_CAN_ID: 5
}

leg_length = [0, 0.13, 0.24, 0.24, 0.13, 0.15]


class ChassisCtrl:
    """
    机器人底盘控制类
    """
    def __init__(self):
        # self.knee = [[KneeMotor(LEFT_FRONT_CAN_ID), KneeMotor(LEFT_BACK_CAN_ID)],
        #              [KneeMotor(RIGHT_FRONT_CAN_ID), KneeMotor(RIGHT_BACK_CAN_ID)]]
        # self.wheel = [WheelMotor(LEFT_WHEEL_CAN_ID), WheelMotor(RIGHT_WHEEL_CAN_ID)]
        self.motors = [KneeMotor(LEFT_FRONT_CAN_ID), KneeMotor(LEFT_BACK_CAN_ID),
                       KneeMotor(RIGHT_FRONT_CAN_ID), KneeMotor(RIGHT_BACK_CAN_ID),
                       WheelMotor(LEFT_WHEEL_CAN_ID), WheelMotor(RIGHT_WHEEL_CAN_ID)]
        self.leg = [LegCtrl(leg_length), LegCtrl(leg_length)]           # leg controller
        self.method = LQR()                                             # control method

    def update_motor(self, msg):
        """
        update motor state according to received message
        :return:
        """
        motor_num = msg2motor.get(msg.id)
        if motor_num is not None:
            self.motors[motor_num].decode(msg.data)


if __name__ == '__main__':
    chassis = ChassisCtrl()
