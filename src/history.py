import numpy as np
import matplotlib.pyplot as plt

class Track:
    def __init__(self):
        self.xs = []
        self.ys = []
        self.color = 'r'

    def add_point(self, point):
        self.xs.append(point[0])
        self.ys.append(point[1])

    def show_track(self):
        plt.plot(self.xs, self.ys)
        plt.show()


    def spline_track(self):
        return



class History:
    def __init__(self):
        self.track = Track()
        self.position = []
        self.theta = []
        self.velocity = []
        self.time = 0

    def display_track(self):
        self.track.show_track()


    def generate_track(self):
        for x in np.linspace(0, 10, 50):
            point = np.array([x, -((x - 5)**2) + 25])
            self.track.add_point(point)

    def append(self, time, curr):
        self.position.append(curr.position)
        self.theta.append(curr.theta)
        self.velocity.append(curr.velocity)
        self.time.append(time)
        