import numpy as np
import math

class pid_control:
    def __init__(self):
        self.kp = 1.25#0.8
        self.ki = 0.005
        self.kd = 0.2#1.15
        self.integral = 0.0
    
    def required_acceleration(self, error, prev_error, interval):
        self.integral += error*interval
        self.derivative = (error-prev_error)/interval
        acc = self.kp*error #+ self.ki*self.integral + self.kd*self.derivative
        return acc