class MotorState:
    def __init__(self):
        self.mode = 0           # 电机模式
        self.pos = 0            # 电机位置
        self.vel = 0            # 电机速度
        self.cur = 0            # 电机电流
        self.vol = 0            # 电机电压
        self.temp = 0           # 电机温度
        
class Motor:
    def __init__(self, id):
        self.id = id 
        self.state = MotorState()
        self.tar_state = MotorState()
        
    