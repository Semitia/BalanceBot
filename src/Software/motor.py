class MotorState:
    def __init__(self):
        self.mode = 0           # 电机模式
        self.pos_sum = 0        # 电机位置累计值
        self.pos = 0            # 电机空间位置      0~2pi
        self.vel = 0            # 电机速度
        self.cur = 0            # 电机电流
        self.vol = 0            # 电机电压
        self.temp = 0           # 电机温度


class GearedMotorState(MotorState):
    """
    减速电机状态
    """
    def __init__(self, ratio=5):
        super(GearedMotorState, self).__init__()
        self.ratio = ratio      # 减速比
        self.pos_sum_g = 0      # 减速后输出轴位置累计值
        self.pos_g = 0          # 减速后输出轴位置
        self.vel_g = 0          # 减速后输出轴速度


class Motor:
    """
    电机类
    """
    def __init__(self, motor_id):
        self.id = motor_id
        self.state = MotorState()
        self.tar_state = MotorState()


class GearedMotor:
    """
    减速电机类
    """
    def __init__(self, motor_id, ratio=5):
        self.id = motor_id
        self.state = GearedMotorState(ratio)
        self.tar_state = GearedMotorState(ratio)


class KneeMotor(GearedMotor):
    """
    使用FOC控制板通信协议的关节电机类
    """
    def __init__(self, motor_id, ratio=5):
        super(KneeMotor, self).__init__(motor_id, ratio)

    def decode(self, data):
        """
        解码数据并更新状态
        :param data:
        :return:
        """
        self.state.mode = data[0]
        self.state.pos = (data[1] << 8) | data[2]
        self.state.vel = (data[3] << 8) | data[4]
        self.state.cur = (data[5] << 8) | data[6]
        self.state.vol = (data[7] << 8) | data[8]
        self.state.temp = data[9]

        self.state.pos_g = self.state.pos / self.state.ratio
        self.state.vel_g = self.state.vel / self.state.ratio


class WheelMotor(Motor):
    """
    使用FOC控制板通信协议的轮子电机类
    """
    def __init__(self, motor_id):
        super(WheelMotor, self).__init__(motor_id)

    def decode(self, data):
        self.state.mode = data[0]
        self.state.pos = (data[1] << 8) | data[2]
        self.state.vel = (data[3] << 8) | data[4]
        self.state.cur = (data[5] << 8) | data[6]
        self.state.vol = (data[7] << 8) | data[8]
        self.state.temp = data[9]

