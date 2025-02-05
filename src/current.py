import numpy as np

dt = 0.1

class Current:  
    '''
    Represents a 'fake vehicle' for quick Pure Pursuit testing
    '''  
    def __init__(self, position=np.array(0.0, 0.0), lookAheadDistance=2.0, velocity=1.0, heading=0.0):
        # Current position of vehicle
        self.position = position  

        # Look Ahead Distance
        self.lookAheadDistance = lookAheadDistance
        
        # Velocity of vehicle
        self.velocity = velocity
        
        # Orientation of vehicle
        self.theta = heading
    
    def update(self, acceleration, delta_theta):
        # Update x & y position using 2d Kinematics
        self.position.x = self.position.x + self.velocity * np.cos(self.theta) * dt
        self.position.y = self.position.y + self.velocity * np.sin(self.theta) * dt

        # Update theta angle 
        self.theta = self.theta + self.velocity * np.tan(delta_theta) * dt

    def distance(self, desired_x, desired_y):
        # pythagorean theroem
        return np.hypot(desired_x - self.position.x, desired_y - self.position.y)