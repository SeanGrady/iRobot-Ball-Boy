import numpy as np
from collections import deque

class CirclesBuffer(length):
    def __init__(self, length):
        self.buff_len = length
        self.buff = deque(maxlen = length)
        self.avg = np.array([0,0,0])

    def add_circle(self, circle):
        self.buff.append(circle)
        self.avg = np.array([sum(i) / len(i) for i in zip(*self.buff)])
        """
        x_tot = y_tot = r_tot = 0
        for circle in self.buff:
            x_tot += circle[0]
            y_tot += circle[1]
            r_tot += circle[2]
        x_avg = x_tot / self.buff_len
        y_avg = y_tot / self.buff_len
        r_avg = r_tot / self.buff_len
        self.avg = np.array([x_avg, y_avg, r_avg])
        """
