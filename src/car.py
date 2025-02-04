import numpy as np

class Car:
    def __init__(self, location=np.array(0.0, 0.0), lookAheadDistance=2.0, velocity=1.0, heading=0.0):
        self.location = location
        self.lookAheadDistance = lookAheadDistance
        self.velocity = velocity
        self.heading = heading