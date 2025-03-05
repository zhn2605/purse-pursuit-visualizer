'''
Brayan Ramos Vicente
Fall 2024
Pure Pursuit Controller for F1TENTH Competition
'''

import numpy as np
import matplotlib.pyplot as plt
import math

Look_Forward = 3.5 #Originally 2
WB = 2.9 # m Distance Between Axles; Wheelbase
dt = 0.1 # Seconds

# For Simulation (Only necessary in case you don't want the simulation to run every time the code runs, like for debugging)
animation = True

class Current:

    def __init__(self, x=0.0, y=0.0, y_w=0.0, v=0.0):
        # Posiiton of Front Axle of Vehicle on xy-plane
        self.front_x = x
        self.front_y = y

        # Vehicle Orientation Angle (Angle Between 'Center Line' of Curve and Direction of Car)
        self.theta = y_w

        # Vehicle Linear Velocity
        self.v = v

        # Position of Rear Axle of Vehicle on xy-plane
        self.rear_x = self.front_x - WB * np.cos(self.theta)
        self.rear_y = self.front_y - WB * np.sin(self.theta)

    def update(self, a, delta):
        # Updates x Position by Adding Velocity * cosine of the steering angle * change in time
        self.front_x = self.front_x + self.v * np.cos(self.theta) * dt  

        # Same for y position
        self.front_y = self.front_y + self.v * np.sin(self.theta) * dt

        # Updating Vehicle Oritentaion Angle Using the Steering Angle for Front Wheels
        self.theta = self.theta + (self.v / WB) * math.tan(delta) * dt # delta is the steering angle, comes from tan(delta) = k(WB)
        # This Equation Comes From theta(dot) = v/R = v/R, R = (WB)/tan(delta)
        
        # Velocity is Updated by Multiplying Acceleration * Change in Time
        # self.v = self.v + a * dt
        self.v = 5;

        # Updating x, y position of rear axle
        self.rear_x = self.front_x - WB * np.cos(self.theta)
        self.rear_y = self.front_y - WB * np.sin(self.theta)

    def distance(self, x_d, y_d):
        # x_d is desired x position
        delta_x = x_d - self.rear_x # For one Specific LookAhead Point

        # y_d is desired y position
        delta_y = y_d - self.rear_y

        # calculate distance from current (x, y) position to new/desired (x, y) position
        return math.hypot(delta_x, delta_y)

class History: 
    
    def __init__(self):
        self.x = [] # x position
        self.y = [] # y position
        self.theta = [] # Orientation Angle
        self.v = [] # Velocity
        self.t = [] # Time

    def append(self, t, state): # Inputs are Time and The State it is in (Most Likely Current)
        self.x.append(state.front_x)
        self.y.append(state.front_y)
        self.theta.append(state.theta)
        self.v.append(state.v)
        self.t.append(t)
        # These Keep Track of the previous x, y, v, theta, and time Points Used (Mainly for Plot)

class CourseManager:
    
    # This Keeps the x,y Points of the Course / Creates the Course from the Points Given
    def __init__(self, course_x, course_y):
        self.cx = course_x
        self.cy = course_y
        self.previous_nearest_point_index = None

    def search(self, state):
        if self.previous_nearest_point_index is None: 
            # For First Run Only
            # These Lists Keeps Distances From Current Position to Remaining Positions on Course
            dx = [] 
            dy = []
            for icx in self.cx:
                dx.append(state.rear_x - icx)
            for icy in self.cy:
                dy.append(state.rear_y - icy)
            d = np.hypot(dx, dy)
            # Find Nearest Point on Course
            index = np.argmin(d)
            # Make The Recent Nearest Point's Index the Previous Nearest Point
            self.previous_nearest_point_index = index
        else:
            # For Every Run After the First
            index = self.previous_nearest_point_index
            distance_to_this_index = state.distance(self.cx[index], self.cy[index])
            while True:
                distance_to_next_index = state.distance(self.cx[index], self.cy[index])
                if distance_to_this_index < distance_to_next_index:
                    break
                if (index + 1) < len(self.cx):
                    index = index + 1
                else:
                    index
                distance_to_this_index = distance_to_next_index
            self.previous_nearest_point_index = index
        # Updating Look Ahead Distance     
        Look_Ahead = np.clip((0.1 * state.v) + Look_Forward, 1, Look_Forward)

        # Searching for Next Look Ahead Point's Index
        while Look_Ahead > state.distance(self.cx[index], self.cy[index]):
            if (index + 1) >= len(self.cx):
                break # So that the Look Ahead Distance Doesn't Exceed Final Point
            index += 1

        return index, Look_Ahead
    
def pure_pursuit_controller(state, course, previous_index):
    # This begins to look for next possible index to begin calculating best path to next look ahead point
    index, Look_Ahead = course.search(state)
    if previous_index >= index:
        index = previous_index

    if index < len(course.cx):
        target_x = course.cx[index]
        target_y = course.cy[index]
    else:
        target_x = course.cx[-1]
        target_y = course.cy[-1]
        index = len(course.cx) - 1

    # Math Needed to Get to Look Ahead Distances(Explained in Guide)
    alpha = math.atan2(target_y - state.rear_y, target_x - state.rear_x) - state.theta

    delta = math.atan2((2.0 * WB * math.sin(alpha)) / Look_Ahead, 1.0)

    return delta, index

def proportional_velocity(target, current):
    # This is so that Velocity Doesn't Get Too High, Causing Instability of Algorithm
    a = 0.1 * (target - current)
    return a

def plot_vehicle(x, y, theta, length=1.0, width=1.0, fc='red', ec='black'): # Placing Vehicle in Simulation and Plot it Along Points Mappped
    if not isinstance(x, float):
        for ix, iy, itheta, in zip(x, y, theta):
            plot_vehicle(ix, iy, itheta)
    else:
        plt.arrow(x, y, length * math.cos(theta), length * math.sin(theta), fc='red', ec='black', head_width=1.00, head_length=1.00)
        plt.plot(x, y)

def main():
    # These Values Create the Curve That Will be Plotted
    curve_x = np.arange(0, 50, 0.1)
    curve_y = []
    for ix in curve_x:
        curve_y.append(math.sin(ix / 5.0) * (ix / 2.0))
    # Speed of Vehicle For Simulation, Can Be Changed
    speed = 10/3.6 # m / s
    
    # Maximum Time of Simulation
    time_max = 50 # s

    # Initial State
    current_state = Current(x=0.0, y=0.0, y_w=0.0, v=0.0)
    
    # Setting Initial Conditions
    previous_Index = len(curve_x) - 1
    current_time = 0.0

    # Creating Arrays for Values
    values = History()
    values.append(current_time, current_state)
    curve = CourseManager(curve_x, curve_y)
    targetted_index, _ = curve.search(current_state)

    while time_max >= current_time and previous_Index > targetted_index:

        #Calculating Control Input
        a = proportional_velocity(speed, current_state.v)
        steering_angle, targetted_index = pure_pursuit_controller(current_state, curve, targetted_index)

        # Updating Current Positions
        current_state.update(10, steering_angle)

        # Updating Time
        current_time += dt
        
        # Adding Current Values to List
        values.append(current_time,current_state)

        if animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_vehicle(current_state.front_x, current_state.front_y, current_state.theta)
            plt.plot(curve_x, curve_y, "-r", label="course")
            plt.plot(values.x, values.y, "-b", label="trajectory")
            plt.plot(curve_x[targetted_index], curve_y[targetted_index], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(current_state.v * 3.6)[:4])
            plt.pause(0.001)

    # Test
    assert previous_Index >= targetted_index, "Cannot goal"

    # Shows Position of the Vehicle on the Graph
    if animation:
        plt.cla()
        plt.plot(curve_x, curve_y, ".r", label="course")
        plt.plot(values.x, values.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        plt.show()

# Calls main and Begins Simulation
if __name__ == '__main__':
    print('\n---Pure Pursuit Begins---\n')
    main()

