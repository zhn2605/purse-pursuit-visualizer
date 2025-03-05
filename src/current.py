import numpy as np
from purePursuit import PurePursuit

dt = 0.1

class Current:  
    '''
    Represents a 'fake vehicle' for quick Pure Pursuit testing
    '''  
    def __init__(self, position=np.array([0.0, 0.0]), lookAheadDistance=2.0, velocity=1.0, heading=0.0, max_accel=2.0):
        # Current position of vehicle
        self.position = position  # (x, y)

        # Look Ahead Distance
        self.lookAheadDistance = lookAheadDistance

        # Look Ahead position
        self.lookAheadPosition = None
        
        # Velocity of vehicle
        self.velocity = velocity
        
        # Orientation of vehicle
        self.theta = heading

        # Acceleration of vehicle
        self.acceleration = 0.0

        # Maximum acceleration
        self.max_accel = max_accel

        
    def distance(self, desired_x, desired_y):
        # pythagorean theroem
        return np.hypot(desired_x - self.position[0], desired_y - self.position[1])

    def update_velocity(self, target_velocity):
        dv = target_velocity - self.velocity
        
        # Implement a clamp for maximum velocity
        if abs(dv) > self.max_accel * dt:
            dv = np.sign(dv) * self.max_accel * dt
        self.velocity += dv

    def update_position(self):
        # Separate into respective components
        dx = self.velocity * np.cos(self.theta) * dt
        dy = self.velocity * np.sin(self.theta) * dt
        
        # Update position
        self.position += np.array([dx, dy])

    def update(self, pure_pursuit: PurePursuit):
        self.lookAheadPosition = pure_pursuit.calc_lookahead_pos(self.position, self.lookAheadDistance)

        if self.lookAheadPosition is not None:
            # Adjust velocity based on distance to look-ahead poin
            target_velocity = min(5.0, 2.0 + self.distance(*self.lookAheadPosition) / 2)
            self.update_velocity(target_velocity)

        self.update_position()