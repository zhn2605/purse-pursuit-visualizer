import numpy as np

class PurePursuit:
    def update(self, car, acceleration, delta_theta):
        # Update x & y position using 2d Kinematics
        car.position.x = car.position.x + car.velocity * np.cos(car.theta) * dt
        car.position.y = car.position.y + car.velocity * np.sin(car.theta) * dt

        # Update theta angle 
        car.theta = car.theta + car.velocity * np.tan(delta_theta) * dt

    # def calc_next_pos(self, car, track):
        # 

    def distance(self, desired_x, desired_y):
        # pythagorean theroem
        return np.hypot(desired_x - self.position.x, desired_y - self.position.y)