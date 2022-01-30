#!/usr/bin/env python3

import rospy
import airsim
from sign_car_recognition.msg import DetectionResult
from std_msgs.msg import Float64
from mapping_navigation.msg import Steer
from lane_keep_assist.msg import LaneStatus
from car_states import *


class CentralControl:

    """"
    This is the central controller of the system bla bla
    """
    def __init__(self):
        #host_ip = rospy.get_param('/host_ip')

        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

        self.state_lane_ticks = 0
        self.lane_state = CarStates.OKAY
        self.next_lane_state = CarStates.OKAY

        self.next_throttle = 0

        self.next_nav_steer = 0

    def listen(self):
        rospy.init_node("central_control", anonymous=True)
        rospy.Subscriber("steering", Float64, self.handle_steering_data)
        # rospy.Subscriber("steering_test", Steer, self.handle_steering_data)
        rospy.Subscriber("lane_info", LaneStatus, self.handle_lane_notification)
        rospy.Subscriber("braking", Float64, self.handle_breaking_data)
        rospy.Subscriber("throttling", Float64, self.handle_throttling_data)
        rospy.Subscriber("sign_detection", DetectionResult, self.handle_sign_recognition)

        rate = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.state_lane_ticks = self.state_lane_ticks + 1

            # If a new bad situation shows up react to it
            if self.lane_state != self.next_lane_state and not self.next_lane_state == CarStates.OKAY:
                self.lane_state = self.next_lane_state
                self.state_lane_ticks = 0

            #Everything is okay, no changes required
            if self.lane_state == CarStates.OFF_LEFT:
                self.next_throttle = self.next_throttle / 2
                rospy.loginfo("OFF_LEFT")
            elif self.lane_state == CarStates.OFF_RIGHT:
                self.next_throttle = self.next_throttle / 2
                rospy.loginfo("OFF_RIGHT")
            elif self.lane_state == CarStates.NO_LANES:
                self.next_throttle = self.next_throttle / 2
                rospy.loginfo("NO_LANES")
            elif self.lane_state == CarStates.OKAY:
                rospy.loginfo("OKAY")

            # Keep the current state for at least a second
            # Pretty much just stops it from going to Okay
            if self.state_lane_ticks == 100:
                self.lane_state = self.next_lane_state
                self.state_lane_ticks = 0

            self.car_controls.throttle = self.next_throttle
            self.car_controls.steer = self.next_nav_steer

            self.client.setCarControls(self.car_controls)
            rate.sleep()

    def handle_steering_data(self, steering_data: Float64):
        self.next_nav_steer = steering_data.data

    def handle_lane_notification(self, lane_info: LaneStatus):
        self.next_lane_state = lane_info.lane_info

    def handle_breaking_data(self, braking_data):
        print("Obtained braking data")

    def handle_throttling_data(self, throttling_data: Float64):
        print("Obtained throttling data")
        self.next_throttle = throttling_data.data

    def handle_sign_recognition(self, sign_data):
        print("Obtained sign recognition data")


if __name__ == "__main__":
    # Do something
    centralControl = CentralControl()
    centralControl.listen()
