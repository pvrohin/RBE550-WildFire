#Code adapted from previous Assignment : Valet
from math import pi, radians, sqrt, tan, sin, cos

from numpy import arange

from obstacle import Obstacle

#Class to track state of vehicle while path planning
class stateTracker():
    def __init__(self, xy_coordinate, theta, psi, x_dot = 0.0, y_dot = 0.0, theta_dot = 0.0, exact = False):
        self.x = xy_coordinate[0]
        self.y = xy_coordinate[1]

        self.theta = theta

        self.theta = self.theta % (2*pi)

        self.psi = psi

        if not exact:
            self.x = round(self.x, 1)
            self.y = round(self.y, 1)
            self.theta = round(self.theta, 1)
            self.psi = round(self.psi, 1)
        self.theta =self.theta % (2*pi) 
        self.psi = self.psi  % (2*pi)

        self.xdot = x_dot
        self.ydot = y_dot
        self.thetadot = theta_dot

        # Ackermann measurements
        self.truck_width = 3 #meters
        self.truck_length = 7 #meters
        self.L = 2.8 # wheelbase, meter
        self.psi_max = radians(60)
        self.max_velocity = 10 # meters / sec

    def get_neighbors(self, time_delta):
        neighbors = []

        psi_increment = radians(15)

        for v in [-self.max_velocity, self.max_velocity]:
            for psi in arange(-self.psi_max, self.psi_max, psi_increment):
                state = self.forward_kinematics(v, psi, time_delta)
                if state.x < 0 or state.x > 250 or state.y < 0 or state.y > 250:
                    continue
                neighbors.append(state)

        return neighbors
    
    def forward_kinematics(self, v, psi, time_delta):
        thetadot = (v/self.L) * tan(psi)
        thetadelta = thetadot * time_delta
        thetadelta = thetadelta % (2*pi)

        if thetadelta > pi:
            thetadelta = (2*pi) - thetadelta
            thetadelta = -1 * thetadelta

        theta = self.theta + thetadelta

        xdot = v * cos(theta)

        xdelta = xdot * time_delta

        ydot = v * sin(theta)

        ydelta = ydot * time_delta

        x = self.x + xdelta
        y = self.y + ydelta

        return stateTracker((x, y), theta, psi, xdot, ydot, thetadot)
    
    def clone(self):
        return stateTracker((self.x, self.y),self.theta,self.psi,exact = True,xdot=self.xdot,ydot=self.ydot,thetadot=self.thetadot)
    
    def transition_cost(self, to):

        distance = self.distance_between_neighbours(to)
        
        theta_difference = abs(self.theta - to.theta)

        if theta_difference > pi:
            theta_difference = (2*pi) - theta_difference

        total_transition_cost = distance + (4*theta_difference)

        return total_transition_cost

    def distance_between_neighbours(self, other):

        x_coordinate = self.x-other.x

        y_coordinate = self.y-other.y

        return sqrt((x_coordinate)**2 + (y_coordinate)**2)

    def calc_obstacle_distance(self, obstacle):

        x_coordinate = self.x-obstacle.xy_coordinate[0]

        y_coordinate = self.y-obstacle.xy_coordinate[1]

        return sqrt((x_coordinate)**2 + (y_coordinate)**2)