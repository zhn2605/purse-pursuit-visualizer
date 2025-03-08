import numpy as np
from purePursuit import PurePursuit

dt = 0.1

class Current:  
    '''
    Represents a 'fake vehicle' for quick Pure Pursuit testing
    '''  
    def __init__(self, position=np.array([0.0, 0.0]), lookAheadDistance=2.0, velocity=1.0, heading=0.0, max_accel=1.0):
        # Current position of vehicle
        self.position = position  # (x, y)

        # Look Ahead Distance
        self.lookAheadDistance = lookAheadDistance

        # Look Ahead position
        self.lookAheadPosition = position
        
        # Velocity of vehicle
        self.velocity = velocity
        
        # Orientation of vehicle
        self.theta = 45
        self.delta_theta = 0

        # Acceleration of vehicle
        self.acceleration = 0.0

        # Maximum acceleration
        self.max_accel = max_accel

        # Curvature
        self.curvature = 0.0

        # Speed limits
        self.max_speed = 10.0
        self.min_speed = 1.0

    def calc_velocity(self, turn_sensitivity=1.0):
        curvature_magnitude = abs(self.curvature)
        # print(curvature_magnitude)
        speed_factor = np.exp(-turn_sensitivity * curvature_magnitude)
        
        target_speed = self.min_speed + (self.max_speed - self.min_speed) * speed_factor

        return target_speed

    def update_velocity(self, target_velocity):
        dv = target_velocity - self.velocity
        
        # Implement a clamp for maximum / minimum acceleration
        acceleration = np.clip(dv / dt, -self.max_accel, self.max_accel)

        self.velocity += acceleration * dt

    def update_position(self):
        # Separate into respective components
        dx = self.velocity * np.cos(self.theta) * dt
        dy = self.velocity * np.sin(self.theta) * dt
        
        # Update position
        self.position += np.array([dx, dy])

    def update(self, pure_pursuit, track):
        self.lookAheadPosition = pure_pursuit.calc_lookahead_pos(self, track)

        # if self.lookAheadPosition is not None:
        #     # Adjust velocity based on distance to look-ahead poin
        #     target_velocity = min(5.0, 2.0 + self.distance(*self.lookAheadPosition) / 2)
        #     self.update_velocity(target_velocity)

        # Use curvature for smoother turns
        self.curvature = pure_pursuit.calc_curvature(self, track)
        self.theta += self.curvature * self.velocity * dt

        # self.delta_theta = pure_pursuit.calc_angle(self, track)

        target_velocity = self.calc_velocity()
        self.update_velocity(target_velocity)
        self.update_position()