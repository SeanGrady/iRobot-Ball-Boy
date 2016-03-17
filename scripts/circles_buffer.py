import numpy as np
from collections import deque

class CirclesBuffer():
    def __init__(self, length):
        self.buff_len = length
        self.buff = deque(maxlen = length)
        self.avg = np.array([0,0,0]).flatten()
        self.circle_bin = deque(maxlen = length)
        self.bin_avg = 0.0

    def add_circle(self, circle):
        self.buff.append(circle)
        self.avg = np.array([sum(i) / len(i) for i in zip(*self.buff)]).flatten()
        self.circle_bin.append(1)
        self.bin_avg = sum(self.circle_bin) / float(self.buff_len)

    def add_empty(self):
        """
        if len(self.buff) > 0:
            self.buff.popleft()
        """
        self.circle_bin.append(0)
        self.bin_avg = sum(self.circle_bin) / float(self.buff_len)


class CirclesStruct():
    def __init__(self, length):
        self.length = length
        self.circles_list = [CirclesBuffer(length)]

    def add_frame_circles(self, circles):
        if circles is not None:
            self.circles_list[0].add_circle(circles[0][0])
        else:
            self.circles_list[0].add_empty()

    def which_circle_is_it(circle):
        pass
