#!/usr/bin/env python3

import rospy
import airsim
from sign_car_recognition.msg import DetectionResult
from std_msgs.msg import Float64, UInt8
from lane_keep_assist.msg import LaneStatus, Lane


class CentralControl:

    """"
    This is the central controller of the system bla bla
    """
    def __init__(self):
        host_ip = rospy.get_param('/host_ip')

        self.client = airsim.CarClient(ip = host_ip)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

    def listen(self):
        rospy.init_node("central_control", anonymous=True)
        rospy.Subscriber("lane_info", LaneStatus, self.handle_lane_data)
        rospy.Subscriber("steering", Float64, self.handle_steering_data)
        rospy.Subscriber("braking", Float64, self.handle_breaking_data)
        rospy.Subscriber("throttling", Float64, self.handle_throttling_data)
        rospy.Subscriber("sign_detection", DetectionResult, self.handle_sign_recognition)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.client.setCarControls(self.car_controls)
            rate.sleep()

    def control(self):
        print("Control loop")

    def handle_steering_data(self, steering_data):
        print("Obtained steering data")
        self.car_controls.steering = steering_data.data

    def handle_breaking_data(self, braking_data):
        print("Obtained braking data")

    def handle_throttling_data(self, throttling_data: Float64):
        print("Obtained throttling data")
        self.car_controls.throttle = throttling_data.data

    def handle_sign_recognition(self, sign_data):
        print("Obtained sign recognition data")

    def handle_lane_data(self, lane_data):
        print("Obtained lane data")
        rospy.loginfo(f"{lane_data.gradient_lanes}")



if __name__ == "__main__":
    # Do something
    centralControl = CentralControl()
    centralControl.listen()
