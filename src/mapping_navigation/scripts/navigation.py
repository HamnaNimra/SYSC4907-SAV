#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from pure_pursuit import *
from std_msgs.msg import Float64
from simple_path import *


class Navigation:

    """
    This does navigation
    """

    def __init__(self, path: List[Point], look_ahead_distance: float, look_forward_gain: float):
        self.steering_pub = rospy.Publisher('steering', Float64, queue_size=10)
        self.path = path
        self.target_ind = 0
        self.navigator = PurePursuit(look_ahead_distance, look_forward_gain, path)
        self.state = State()

    def listener(self):
        rospy.init_node('talker', anonymous=True)
        rospy.Subscriber('airsimPose', PoseStamped, self.handle_gps_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)        
        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            self.target_ind, _ = pure_pursuit.search_target_index(state)
            if len(path)-1 > self.target_ind:
                steering_angle = self.get_steering_angle()
                self.steering_pub(steering_pub)

    def handle_gps_data(self, postition: PoseStamped):
        curr_point = Point((curr_point.pose.position.x, curr_point.pose.position.y))
        quaterion = (curr_point.pose.orientation.x, curr_point.pose.orientation.y,
                     curr_point.pose.orientation.z, curr_point.pose.orientation.w)
        self.state.update_pos(curr_point, quaterion)

    def handle_speed_data(self, speed: Float64):
        self.state.update_speed(speed.data)

    def get_steering_angle(self) -> float:
        ackerman_angle_rad, target_ind = navigator.pure_pursuit_steer_control(state)
        ackerman_angle_def = math.degrees(ackerman_angle_rad) / 45

if __name__ == "__main__":
    #Current route stub
    path = read_points('src\mapping_navigation\scripts\coords.txt')
    navigation = Navigation(look_ahead_distance=4.0, look_forward_gain=0.1, path=path)
    navigation.listener()
