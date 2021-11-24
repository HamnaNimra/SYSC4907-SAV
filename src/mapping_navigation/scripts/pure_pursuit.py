"""
Code adapted from

Path tracking simulation with pure pursuit steering and PID speed control.
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import numpy as np
import math
from typing import List, Tuple
from map_information import *

class State:

    """
    Contains the car information needed in pure pursuit
    The cartesian coordinates for location
    The yaw of the car
    The cars wheel base
    """
    def __init__(self, wheel_base: float, x: float=0.0, y:float=0.0, yaw:float=0.0, velocity:float=0.0):
        self.wheel_base = wheel_base
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity
        self.rear_x = self.x - ((wheel_base / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((wheel_base / 2) * math.sin(self.yaw))
    
    """
    Updates the cars position
    Cartesian coordinates for the cars position
    Quaterion to find the yaw of the car
    """
    def update(self, x: float, y: float, quaterion: Tuple[float, float, float, float], velocity:float):
        self.x = x
        self.y = y
        self.velocity = velocity
        self.yaw = euler_from_quaternion(quaterion)
        self.rear_x = self.x - ((self.wheel_base / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((self.wheel_base / 2) * math.sin(self.yaw))

    """
    Find the distance between the cars current location and another point
    Find the euclidian distance
    """
    def calc_distance(self, point_x:float, point_y:float) -> float:
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class PurePursuit:

    """
    look aheaed distiance is the distance pur pursuit scans to find goals points
    look forward gain is used to increase the look ahead distance based on the cars current speed
    path is the path the algorithm must track
    """
    def __init__(self, look_ahead_distance: float, look_forward_gain: float, path :List[Point]):
        self.look_forward_gain = look_forward_gain
        self.look_ahead_distance = look_ahead_distance
        self.path = path
        #Needs to be refactored to look at our points instead of his cx and cy
        self.old_nearest_point_index = None
        self.old_steer_target = 0
        self.cx = []
        self.cy = []
        for point in path:
            self.cx.append(point.coordinate[0])
            self.cy.append(point.coordinate[1])

    """
    Returns the steering angle in radians and the index to the goal points its turning to
    Takes in the current state of the car
    """
    def pure_pursuit_steer_control(self, state: State) -> Tuple[float, int]:
        
        # Getting the next point
        ind, Lf = self.search_target_index(state)

        # Checks to see if the previous found goal points is actually ahead of what it sees now
        # If the previous index is ahead it uses that over the new one.
        if self.old_steer_target >= ind:
            ind = self.old_steer_target

        if ind < len(self.cx):
            tx = self.cx[ind]
            ty = self.cy[ind]
        else:  # toward goal
            tx = self.cx[-1]
            ty = self.cy[-1]
            ind = len(self.cx) - 1

        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
        delta = math.atan2(2.0 * state.wheel_base * math.sin(alpha) / Lf, 1.0)
        self.old_steer_target = ind

        return delta, ind

    """
    Finds the next goal point given the cars state.
    Adjusts the looks forward distance based on the cars speed.
    """
    def search_target_index(self, state: State) -> Tuple[int, float]:

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            # Mapping though all points in the path, and finding the difference in height and width to that point.
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            #Getting the minimum hypotenues, aka the nearest point
            #For this firt time set it to the closest point.
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            #Get distance between the nearest point we found last time and the cars current posotiion.
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                #Get the distance from the cars current posotion to the next point after the nearest point
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                #If the closest point is still the nearest point we found earlier exit the loop
                if distance_this_index < distance_next_index:
                    break
                # If the next next point is actually closer go to that instead and reapeat the calculation
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        #Finds where we should actually be looking taking into acound the vehicles current speed
        #look ahead distance=look ahead gain*vehicle forward velocity
        Lf = self.look_forward_gain * state.velocity + self.look_ahead_distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

# Return yaw in radians from a quaterion
def euler_from_quaternion(quaterion: Tuple[float, float, float, float]) -> float:
        x, y, z, w = quaterion

        siny_cosp = 2 * (w*z + x*y)
        cosy_cosp = 1 - 2 * (y*y + z*z)
        yaw_z = math.atan2(siny_cosp, cosy_cosp)

        return yaw_z
