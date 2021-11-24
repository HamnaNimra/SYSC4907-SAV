#!/usr/bin/env python3

import airsim
import numpy as np
from pure_pursuit import *
from simple_path import *
import math
from pure_pursuit import *


def main():

    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)

    #Get coordinates
    path = read_points('coords.txt')

    pure_pursuit = PurePursuit(look_ahead_distance=4.0, look_forward_gain=0.1, path=path)

    # initial state
    state = State( wheel_base=2.2, x=0.0, y=0.0, yaw=0.0, velocity=0.0)

    #Getting the index to the first point
    target_ind, _ = pure_pursuit.search_target_index(state)

    while len(path)-1 > target_ind:

        # Calc control input
        delta, target_ind = pure_pursuit.pure_pursuit_steer_control(state)

        car_state = client.getCarState()
        pos = car_state.kinematics_estimated.position
        orientation = car_state.kinematics_estimated.orientation
        quaterion = (orientation.x_val, orientation.y_val, orientation.z_val, orientation.w_val)

        car_controls = airsim.CarControls()
        car_controls.throttle = .5
        #Estimating the steering ratio atm
        car_controls.steering = math.degrees(delta) / 45
        client.setCarControls(car_controls)

        state.update(pos.x_val ,pos.y_val ,quaterion, car_state.speed)  # Control vehicle

if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()