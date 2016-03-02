import numpy as np
from collections import deque

class CirclesBuffer(length):
    def __init__(self, length):
        self.buff = deque(maxlen = length)
        self.avg = np.array([0,0,0])
