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


class CirclesStruct():
    def __init__(self):
        self.circles = [CirclesBuffer()]

    def add_frame_circles(self, circles):
        pass

    def which_circle_is_it(circle):
        pass
