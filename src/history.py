import numpy as np

class History:
    def __init__(self):
        self.position = []
        self.theta = []
        self.velocity = []
        self.time = []

    def append(self, time, curr):
        self.position.append(curr.position)
        self.theta.append(curr.theta)
        self.velocity.append(curr.velocity)
        self.time.append(time)
        