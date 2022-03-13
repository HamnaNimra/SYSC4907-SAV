#!/usr/bin/env python3

# Temp file to read in coordinates from the car, get a new point every 2 meters.
import numpy as np
import airsim
from map_information import *
from typing import List


# Write points to a file, can set the total distance and the distance between the points
def write_points(filename: str, dist: int, delta: int):
    client = airsim.CarClient()
    client.confirmConnection()

    with open(filename, "w") as coord_file:

        # Get the starting point
        pos_init = client.getCarState().kinematics_estimated.position
        x_init = np.float16(pos_init.x_val)
        y_init = np.float16(pos_init.y_val)
        last_point = np.array(x_init, y_init)

        # Run for dist number of meters
        for i in range(dist):
            new_point = False

            # poll unit a new point is recorded
            while not new_point:
                pos_curr = client.getCarState().kinematics_estimated.position
                x_curr = np.float16(pos_curr.x_val)
                y_curr = np.float16(pos_curr.y_val)
                curr_point = np.array(x_curr, y_curr)

                # Euclidean distance
                dist = np.linalg.norm(curr_point - last_point)

                # record point if distance is greater than set delta
                if dist >= delta:
                    new_point = True
                    coord_file.write(f'{x_curr}:{y_curr}\n')
                    last_point = curr_point


# Read in points to a file as a list
def read_points(filename: str) -> List[Point]:
    points_list = []
    with open(filename) as coord_file:
        str_points = coord_file.readlines()
        for str_point in str_points:
            x, y = tuple(str_point.split(':'))
            points_list.append(Point((float(x), float(y))))
    return points_list
