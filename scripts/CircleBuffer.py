from collections import deque

class CircleBuffer(buff_len):
    def __init__(self):
        self.buff_len = buff_len
        self.average = 0.0
        self.buff = deque(maxlen=self.buff_len)
