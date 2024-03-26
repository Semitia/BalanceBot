# PID controller
import time

class PID:
    def __init__(self, P, I, D, max_output,fw,I_max):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.max_output = max_output
        self.feed_forward = fw
        self.sample_time = 0.00
        self.current_time = time.time()
        self.last_time = self.current_time
        self.I_MAN = I_max
        self.clear()
        
    def clear(self):
        self.SetPoint = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.int_error = 0.0
        self.output = 0.0
        

    def set_target(self, target):
        self.SetPoint = target
        # self.ITerm = 0
        # self.last_error = 0
        self.last_time = time.time()

    def cal_output(self, real_value):
        error = self.SetPoint - real_value
        self.current_time = time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error
        
        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time
            
            # I
            if (self.ITerm < -self.I_MAN):
                self.ITerm = -self.I_MAN
            elif (self.ITerm > self.I_MAN):
                self.ITerm = self.I_MAN
                
            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time
                
            self.last_time = self.current_time
            self.last_error = error
            
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)
            self.output = max(min(self.output, self.max_output), -self.max_output)
            # print("dt , PID PTerm, ITerm, DTerm, output", delta_time, self.PTerm, self.ITerm, self.DTerm, self.output)
            
    def setKp(self, proportional_gain):
        self.Kp = proportional_gain
        
    def setKi(self, integral_gain):
        self.Ki = integral_gain
        
    def setKd(self, derivative_gain):
        self.Kd = derivative_gain
    
        
    def setSampleTime(self, sample_time):
        self.sample_time = sample_time