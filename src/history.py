import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.animation import FuncAnimation

class Track:
    def __init__(self):
        self.xs = []  # x coordinates
        self.ys = []  # y coordinates
        self.color = 'k'

    def add_point(self, point):
        self.xs.append(point[0])
        self.ys.append(point[1])
    
    def show_track(self):
        plt.plot(self.points[:, 0], self.points[:, 1], color='k', marker='o', linestyle='-')
        plt.show()

    # Will work on this later
 #def spline_track(self):
# if len(self.points) < 3
# print("Not enough points for spline")
# return
#
# x, y = self.points[:, 0], self.points[:, 1]
# cubic_spline = CubicSpline(x, y)
#

class History:
    def __init__(self):
        self.track = Track()
        self.position = []
        self.theta = []
        self.velocity = []
        self.time = []
    
    def display_track(self):
        self.track.show_track()
    
    def generate_track(self):
        for x in np.linspace(0, 10, 50):
            point = np.array([x, -((x - 5)**2) + 25])
            self.track.add_point(point)
    
    def append(self, time, curr):
        self.position.append(curr.position.copy())
        self.theta.append(curr.theta)
        self.velocity.append(curr.velocity)
        self.time.append(time)
    
    def animate(self, car, pure_pursuit, interval=50):        
        fig, ax = plt.subplots()
        ax.set_xlim(min(self.track.xs) - 2, max(self.track.xs) + 2)  # Set x-axis limits
        ax.set_ylim(min(self.track.ys) - 5, max(self.track.ys) + 5)  # Set y-axis limits
        ax.plot(self.track.xs, self.track.ys, color=self.track.color, label="Track")  # Draw the static track
    
        # Store trajectory points
        trajectory_xs = []
        trajectory_ys = []
        trajectory_line, = ax.plot([], [], '-b', label="Trajectory")  # Trajectory line
    
        car_dot, = ax.plot([], [], 'bo', markersize=8, label="Car")
        lookAheadDot, = ax.plot([], [], 'kx', markersize=10, label="Lookahead Point")

        def update(frame):
            # Update angle and position
            delta_theta = pure_pursuit.calc_angle(car, self.track)
            car.update(pure_pursuit, self.track)
            car.lookAheadPosition = pure_pursuit.calc_lookahead_pos(car, self.track)

            # Update and store trajectory
            trajectory_xs.append(car.position[0])
            trajectory_ys.append(car.position[1])

            # Update plot data
            car_dot.set_data([car.position[0]], [car.position[1]])
            lookAheadDot.set_data([car.lookAheadPosition[0]], [car.lookAheadPosition[1]])
            trajectory_line.set_data(trajectory_xs, trajectory_ys)  # Update trajectory line

            print("Car Position:", car.position)
            print("Lookahead Position:", car.lookAheadPosition)

            return car_dot, lookAheadDot, trajectory_line

        ani = animation.FuncAnimation(fig, update, frames=100, interval=interval, blit=True)

        plt.legend()
        plt.grid(True)
        plt.show()