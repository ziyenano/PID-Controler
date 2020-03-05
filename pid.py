#!coding:utf8
import numpy as np
from matplotlib import pyplot as plt

class PID_Controller:
    """
    PID controller demo
    """
    def __init__(self, xd = 1.0, x0 = 0.1, size = 100):
        """
        initialize some necessary parameter
        """
        self.xd = xd
        self.x0 = x0
        self.size = size
        self._axis_y = np.zeros([size, 1])
        self._axis_x = np.zeros([size, 1])

    def P_control(self, kp, c = 0):
        """
        P controller
        """
        idx = 0
        x = self.x0
        xd = self.xd
        while idx < self.size:
            self._axis_y[idx] = x
            self._axis_x[idx] = idx
            e = xd - x
            u = kp * e
            x += u - c
            idx += 1
    
    def PI_control(self, kp, ti, c = 0):
        """
        PI controller
        """
        int_e = 0
        idx = 0
        x = self.x0
        xd = self.xd
        while idx < self.size:
            self._axis_y[idx] = x
            self._axis_x[idx] = idx
            e = xd - x
            int_e += e
            u = kp * e + kp/ti * int_e
            x += u - c
            idx += 1

    def PID_control(self, kp, ti, c_fun, tau = 0):
        """
        PID controller
        """
        int_e = 0
        idx = 0
        x = self.x0
        xd = self.xd
        de = 0
        while idx < self.size:
            self._axis_y[idx] = x
            self._axis_x[idx] = idx
            e = xd - x
            int_e += e
            de = (-kp * e - kp / ti * int_e + c_fun(idx)) / (1 + kp * tau)
            u = kp * e + kp / ti * int_e + kp * tau * de
            x += u - c_fun(idx)
            idx += 1

    def plot(self):
        plt.plot(self._axis_x, self._axis_y)
        plt.show()
