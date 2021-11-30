#!/usr/bin/env python3

import rospy
import os
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from pure_pursuit import *
from std_msgs.msg import Float64
from simple_path import *


class Navigation:

    """
    This does navigation
    """

    def __init__(self, path: List[Point], look_ahead_distance: float, look_forward_gain: float, wheel_base: float):
        self.steering_pub = rospy.Publisher('steering', Float64, queue_size=10)
        self.path = path
        self.target_index = 0
        self.navigator = PurePursuit(look_ahead_distance, look_forward_gain, path)
        self.state = State(wheel_base)

    def listener(self):
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber('airsimPose', PoseStamped, self.handle_gps_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)        
        rate = rospy.Rate(30)
        # Once to get initial starting index
        self.target_index, _ = self.navigator.search_target_index(self.state)

        while not rospy.is_shutdown():
            if len(self.path)-1 > self.target_index:
                steering_angle = self.get_steering_angle()
                self.steering_pub.publish(steering_angle)
            else:
                rospy.loginfo("Done route")
            rate.sleep()

    def handle_gps_data(self, postition: PoseStamped):
        curr_point = Point((postition.pose.position.x, postition.pose.position.y))
        quaterion = (postition.pose.orientation.x, postition.pose.orientation.y,
                     postition.pose.orientation.z, postition.pose.orientation.w)
        self.state.update_pos(curr_point, quaterion)

    def handle_speed_data(self, speed: Float64):
        self.state.update_speed(speed.data)

    def get_steering_angle(self) -> float:
        ackerman_angle_rad, self.target_index = self.navigator.pure_pursuit_steer_control(self.state)
        ackerman_angle_def = math.degrees(ackerman_angle_rad) / 45
        return ackerman_angle_def

if __name__ == "__main__":
    #Current route stub enter the absolute path to the file here
    path = read_points(os.path.abspath(os.path.dirname(__file__)) + "/coords.txt")
    navigation = Navigation(look_ahead_distance=4.0, look_forward_gain=0.1, path=path, wheel_base=2.2)
    navigation.listener()
